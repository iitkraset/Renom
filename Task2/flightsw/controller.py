"""MK-1A flight software - Python port of Core/Src/main.c.

Structure is kept 1:1 with the C firmware: init, ground calibration, then a fixed-rate 100 Hz
superloop that reads the sensors, runs the altitude filter, drives the fins, runs the flight
state machine, writes the SD log, saves flight state to flash and sends radio telemetry.

Entry point: main(board). board is the MK-1A hardware (see README.md for the API).
"""
import math
import struct

# ---- Settings (same values as main.c) --------------------------------------------------------
LOOP_PERIOD_MS = 10            # 100 Hz control/sensor loop
TM_SAMPLE_PERIOD_MS = 125      # telemetry sample rate: 8 Hz
TM_SAMPLES_PER_FRAME = 8       # 8 samples per radio frame -> 1 frame/s
DIAG_PERIOD_MS = 1000
TEAM_NUMBER = 30
RAD_TO_DEG = 57.29578
IMU_STALE_MS = 100
SD_SYNC_PERIOD_MS = 1000
CONFIRM_SAMPLES = 5
LOG_PERIOD_MS = 50             # SD CSV log rate: 20 Hz
FSTORE_PERIOD_MS = 1000
LOG_RETRY_PERIOD_MS = 2000
GPS_POLL_TIMEOUT_MS = 2

AB_ALPHA = 0.1
AB_BETA = 0.005

STATE_PAD, STATE_BOOST, STATE_COAST, STATE_DESCENT, STATE_LANDED = 0, 1, 2, 3, 4
STATE_NAMES = ("PAD", "BOOST", "COAST", "DESCENT", "LANDED")

LAUNCH_ACCEL_THRESHOLD = 30.0
LAUNCH_CONFIRM_SAMPLES = 20
LAUNCH_DETECT_ALT = 50.0
LAUNCH_ABORT_TIME = 3000
LAUNCH_ABORT_ALT = 20.0
GROUND_TRACK_TAU_S = 60.0

BURNOUT_ACCEL_THRESHOLD = 15.0
BURNOUT_SPEED_DROP = 10.0
MIN_BOOST_TIME = 100
MAX_BOOST_TIME = 4000

MIN_COAST_TIME = 1000
MAX_COAST_TIME = 12500
TIMER_MAX_CLIMB_SPEED = 2.0
APOGEE_SPEED_THRESHOLD = 1.0
APOGEE_ABS_THRESHOLD = 0.5
APOGEE_ALT_DROP = 5.0
MIN_DEPLOY_ALT = 30.0

PYRO_ON_TIME = 1000
BARO_TIMEOUT = 200
LANDING_SPEED_THRESHOLD = 1.0
LANDING_STABLE_TIME = 3000
BARO_MAX_RESID = 20.0
BARO_REINIT_GAP = 100
NVM_MAINTAIN_FRACTION = 0.75   # erase the record sector on the ground once it is this full

LOG_HEADER = (
    "time_s,loop,state,state_name,baro_healthy,imu_ok,deploy_allowed,pyro1,baro_good,loop_work_ms,"
    "alt_raw_m,alt_m,speed_mps,press_pa,temp_c,ground_press_pa,max_alt_m,max_speed_mps,"
    "ax_mps2,ay_mps2,az_mps2,accel_mag_mps2,roll_deg,pitch_deg,yaw_deg,gz_dps,vdda_v,"
    "gps_time,gps_lat,gps_lon,gps_alt_m,gps_sats,servo1,servo2,overruns,imu_errors,baro_rejects,log_errors\r\n"
)


# ---- Helpers ---------------------------------------------------------------------------------
def find_altitude(ground_pa, pa):
    """BMP388_FindAltitude: metres above the ground reference."""
    return 44330.0 * (1.0 - (pa / ground_pa) ** (1.0 / 5.255))


def quat_to_euler(q):
    """bno085_quatToEuler: returns roll, pitch, yaw in degrees."""
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, s)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG


def gps_validate(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cks = line[1:].partition("*")
    c = 0
    for ch in body:
        c ^= ord(ch)
    return cks[:2].upper() == "%02X" % c


def nmea_to_decimal(v):
    deg = int(v / 100)
    return deg + (v - deg * 100) / 60.0


def crc32_ieee(data):
    """CRC-32 (IEEE 802.3), bitwise like crc32Ieee() in main.c."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1) & 0xFFFFFFFF))
    return crc ^ 0xFFFFFFFF


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def scale_i(x, k, lo, hi):
    v = x * k
    if v != v:
        return 0
    return int(clamp(round(v), lo, hi))


# ---- Flight software ---------------------------------------------------------------------------
class Flight:
    def __init__(self, board):
        self.b = board
        self.state = STATE_PAD
        self.state_entry_time = 0
        self.confirm_count = 0
        self.launch_tick = 0
        self.max_alt = 0.0
        self.max_speed = 0.0
        self.curr_height = 0.0
        self.curr_speed = 0.0
        self.ground_press = 0.0
        self.press = 0.0
        self.temp = 0.0
        self.baro_alt_raw = 0.0
        self.last_baro_ok = 0
        self.baro_healthy = False
        self.baro_rejects = 0
        self.pyro1_tick = 0
        self.landing_since = 0
        self.log_open = False
        self.log_count = 0
        self.log_errors = 0
        self.loop_overruns = 0
        self.loop_work_ms = 0
        self.packet_count = 0
        self.tm_samples = []
        self.tm_base_time = 0
        self.tm_base_count = 0
        self.tm_frames_sent = 0
        self.imu = None
        self.imu_errors = 0
        self.euler = (0.0, 0.0, 0.0)
        self.voltage = 0.0
        self.gps_time = 0.0
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_alt = 0.0
        self.gps_sats = 0
        self.ccr = (400, 400)

    # -- flash store (fstore.c) --
    def fstore_save(self):
        rec = struct.pack("<BffffI", self.state, self.ground_press, self.max_alt, self.max_speed,
                          self.curr_height, self.b.millis())
        self.b.nvm.save(rec)

    def enter_state(self, s):
        self.state = s
        self.state_entry_time = self.b.millis()
        self.confirm_count = 0
        if s == STATE_BOOST:
            self.launch_tick = self.state_entry_time
            self.max_speed = 0.0
            self.max_alt = 0.0
        self.fstore_save()      # survive a power cut

    def confirmed_for(self, cond, samples):
        if not cond:
            self.confirm_count = 0
        elif self.confirm_count < 255:
            self.confirm_count += 1
        return self.confirm_count >= samples

    def confirmed(self, cond):
        return self.confirmed_for(cond, CONFIRM_SAMPLES)

    # -- GPS (HAL_UART_RxCpltCallback + gpsParse) --
    def gps_poll(self):
        line = self.b.gps.readline(timeout_ms=GPS_POLL_TIMEOUT_MS)
        if not line:
            return
        s = line.decode("ascii", "replace").strip()
        if not gps_validate(s):
            return
        f = s.split("*")[0].split(",")
        try:
            if f[0] == "$GNGGA":
                self.gps_time = float(f[1])
                self.gps_lat = nmea_to_decimal(float(f[2]))
                self.gps_lon = nmea_to_decimal(float(f[4]))
                self.gps_sats = int(f[7])
                self.gps_alt = float(f[9])
            elif f[0] == "$GNRMC":
                self.gps_time = float(f[1])
                self.gps_lat = nmea_to_decimal(float(f[3]))
                self.gps_lon = nmea_to_decimal(float(f[5]))
        except (ValueError, IndexError):
            pass

    # -- SD log --
    def log_open_new(self):
        try:
            self.b.sd.open_new()
            self.b.sd.write(LOG_HEADER.encode())
            self.b.sd.sync()
            return True
        except Exception:
            return False

    def log_line(self, now, imu_ok, deploy_allowed, baro_good, accel, accel_mag):
        roll, pitch, yaw = self.euler
        gz = self.imu.gyro.z * RAD_TO_DEG if self.imu else 0.0
        line = (
            f"{now / 1000:.3f},{self.log_count},{self.state},{STATE_NAMES[self.state]},"
            f"{int(self.baro_healthy)},{int(imu_ok)},{int(deploy_allowed)},{self.b.pyro.state(1)},"
            f"{int(baro_good)},{self.loop_work_ms},"
            f"{self.baro_alt_raw:.3f},{self.curr_height:.3f},{self.curr_speed:.3f},{self.press:.2f},"
            f"{self.temp:.2f},{self.ground_press:.2f},{self.max_alt:.3f},{self.max_speed:.3f},"
            f"{accel[0]:.4f},{accel[1]:.4f},{accel[2]:.4f},{accel_mag:.4f},"
            f"{roll:.3f},{pitch:.3f},{yaw:.3f},{gz:.3f},{self.voltage:.3f},"
            f"{self.gps_time:.2f},{self.gps_lat:.7f},{self.gps_lon:.7f},{self.gps_alt:.2f},{self.gps_sats},"
            f"{self.ccr[0]},{self.ccr[1]},{self.loop_overruns},{self.imu_errors},{self.baro_rejects},"
            f"{self.log_errors}\r\n"
        )
        try:
            self.b.sd.write(line.encode())
            self.log_count += 1
        except Exception:
            self.log_open = False
            self.log_errors += 1

    # -- Telemetry (tmTakeSample / tmBuildFrame / tmSendFrame) --
    def tm_take_sample(self, t_ms, accel):
        if not self.tm_samples:
            self.tm_base_time = t_ms
            self.tm_base_count = self.packet_count
        roll, pitch, _ = self.euler
        g = self.imu.gyro if self.imu else None
        v12 = [scale_i(accel[0], 10, -2048, 2047), scale_i(accel[1], 10, -2048, 2047),
               scale_i(accel[2], 10, -2048, 2047), scale_i(roll, 10, -2048, 2047),
               scale_i(pitch, 10, -2048, 2047)]
        for c in ((g.x, g.y, g.z) if g else (0.0, 0.0, 0.0)):
            v12.append(scale_i(c * RAD_TO_DEG, 1, -2048, 2047))
        self.tm_samples.append((self.state, scale_i(self.curr_height, 10, -2**31, 2**31 - 1),
                                int(self.press + 0.5) if self.press > 0 else 0, v12))
        self.packet_count += 1

    def tm_build_frame(self):
        b0 = self.tm_samples[0]
        out = bytearray(b"\xaa\x55")
        out += struct.pack("<BB", 3, TEAM_NUMBER)
        out += struct.pack("<IBIiI", self.tm_base_count, len(self.tm_samples), self.tm_base_time, b0[1], b0[2])
        out += struct.pack("<hH", scale_i(self.temp, 10, -32768, 32767), int(self.voltage * 100 + 0.5))
        out += struct.pack("<iiiB", scale_i(self.gps_lat, 1e7, -2**31, 2**31 - 1),
                           scale_i(self.gps_lon, 1e7, -2**31, 2**31 - 1),
                           scale_i(self.gps_alt, 10, -2**31, 2**31 - 1), clamp(self.gps_sats, 0, 255))
        for (st, alt, pr, v12) in self.tm_samples:
            out += struct.pack("<Bhh", st, clamp(alt - b0[1], -32768, 32767), clamp(pr - b0[2], -32768, 32767))
            bits = 0
            for k, v in enumerate(v12):
                bits |= (v & 0xFFF) << (12 * k)
            out += bits.to_bytes(12, "little")
        out += struct.pack("<I", crc32_ieee(out[2:]))
        return bytes(out)

    def tm_send_frame(self):
        if not self.tm_samples:
            return
        frame = self.tm_build_frame()
        self.b.radio.write(frame)
        self.tm_frames_sent += 1
        self.tm_samples = []

    # -- Startup --
    def init(self):
        b = self.b
        b.imu.begin()
        b.baro.begin()
        b.delay_ms(10)
        self.log_open = b.sd.mount() and self.log_open_new()
        # Ground calibration: average 50 baro samples
        acc, n = 0.0, 0
        for i in range(70):
            b.imu.service()
            d = b.baro.read()
            if d and i >= 20 and 30000 < d.pressure < 110000:
                acc += d.pressure
                n += 1
            b.delay_ms(20)
        self.ground_press = acc / max(n, 1)
        self.press = self.ground_press
        if b.nvm.used() >= NVM_MAINTAIN_FRACTION * b.nvm.capacity():
            b.nvm.erase()   # stalls ~0.4 s, so only on the ground
        self.state = STATE_PAD
        self.fstore_save()

    # -- Main loop --
    def run(self):
        b = self.b
        self.init()
        last_tick = b.millis()
        next_tick = last_tick + LOOP_PERIOD_MS
        last_diag = last_sync = last_log = last_fstore = last_retry = last_tm = last_tick
        last_imu_recover = 0
        ground_sum, ground_n, last_ground = 0.0, 0, last_tick
        accel = (0.0, 0.0, 9.81)

        while b.running():
            # Fixed-rate scheduler: wait for the next 10 ms slot
            while b.millis() < next_tick:
                pass
            now = b.millis()
            if now - next_tick >= LOOP_PERIOD_MS:
                self.loop_overruns += 1
                next_tick = now + LOOP_PERIOD_MS
            else:
                next_tick += LOOP_PERIOD_MS
            dt = (now - last_tick) / 1000.0
            last_tick = now

            # --- Sensors ---
            d = b.imu.service()
            if d is None:
                self.imu_errors += 1
                if now - last_imu_recover >= 100:
                    last_imu_recover = now
                    b.imu.recover()
            else:
                self.imu = d
                if d.new:
                    self.imu_tick = now
            if self.imu is not None:
                self.euler = quat_to_euler(self.imu.quat)
                accel = (self.imu.accel.x, self.imu.accel.y, self.imu.accel.z)
            imu_age = now - getattr(self, "imu_tick", -10**9)
            imu_ok = imu_age < IMU_STALE_MS
            accel_mag = math.sqrt(accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2)

            bd = b.baro.read()
            self.voltage = b.adc.vdda()
            self.gps_poll()

            # Alpha-beta filter
            alt_pred = self.curr_height + self.curr_speed * dt
            baro_good = False
            if bd is not None:
                p, t = bd.pressure, bd.temperature
                alt_meas = find_altitude(self.ground_press, p) if p > 0 else float("nan")
                resid = alt_meas - alt_pred
                recent = self.last_baro_ok != 0 and now - self.last_baro_ok < BARO_REINIT_GAP
                if p < 30000.0 or p > 110000.0 or alt_meas != alt_meas:
                    self.baro_rejects += 1
                elif recent and abs(resid) > BARO_MAX_RESID:
                    self.baro_rejects += 1
                else:
                    baro_good = True
                    self.press, self.temp, self.baro_alt_raw = p, t, alt_meas
                    if not recent and self.last_baro_ok != 0:
                        self.curr_height = alt_meas
                    else:
                        self.curr_height = alt_pred + AB_ALPHA * resid
                        if dt > 0:
                            self.curr_speed += (AB_BETA / dt) * resid
                    self.last_baro_ok = now
            if not baro_good:
                self.curr_height = alt_pred
            self.baro_healthy = self.last_baro_ok != 0 and now - self.last_baro_ok < BARO_TIMEOUT

            # --- Fin control ---
            yaw = self.euler[2]
            if yaw > 10:
                self.ccr = (250 + 5 * 60, 250 + 5 * 0)
            elif yaw < -10:
                self.ccr = (250 + 5 * 0, 250 + 5 * 60)
            else:
                self.ccr = (250 + 5 * 30, 250 + 5 * 30)
            b.fins.set(self.ccr[0], self.ccr[1], stamp=self.imu.stamp if self.imu else None)

            # --- Flight state machine ---
            tis = now - self.state_entry_time
            if self.state in (STATE_BOOST, STATE_COAST):
                self.max_speed = max(self.max_speed, self.curr_speed)
                self.max_alt = max(self.max_alt, self.curr_height)
                if self.baro_healthy and now - self.launch_tick > LAUNCH_ABORT_TIME and self.max_alt < LAUNCH_ABORT_ALT:
                    self.enter_state(STATE_PAD)
            if self.state in (STATE_BOOST, STATE_COAST) and now - last_fstore >= FSTORE_PERIOD_MS:
                last_fstore = now
                self.fstore_save()
            deploy_allowed = self.max_alt >= MIN_DEPLOY_ALT or not self.baro_healthy

            if self.state == STATE_PAD:
                if baro_good and abs(self.curr_speed) < 2.0 and self.curr_height < 10.0:
                    ground_sum += self.press
                    ground_n += 1
                if now - last_ground >= 1000:
                    if ground_n > 50:
                        step = (now - last_ground) / 1000.0 / GROUND_TRACK_TAU_S
                        self.ground_press += (ground_sum / ground_n - self.ground_press) * step
                    ground_sum, ground_n, last_ground = 0.0, 0, now
                if self.confirmed_for((imu_ok and accel_mag > LAUNCH_ACCEL_THRESHOLD) or
                                      (self.baro_healthy and self.curr_height > LAUNCH_DETECT_ALT),
                                      LAUNCH_CONFIRM_SAMPLES):
                    self.enter_state(STATE_BOOST)
            elif self.state == STATE_BOOST:
                if tis > MAX_BOOST_TIME:
                    self.enter_state(STATE_COAST)
                elif tis >= MIN_BOOST_TIME and self.confirmed(
                        (imu_ok and accel_mag < BURNOUT_ACCEL_THRESHOLD) or
                        (self.baro_healthy and 0 < self.curr_speed < self.max_speed - BURNOUT_SPEED_DROP)):
                    self.enter_state(STATE_COAST)
            elif self.state == STATE_COAST:
                if tis > MAX_COAST_TIME and deploy_allowed and \
                        (not self.baro_healthy or self.curr_speed < TIMER_MAX_CLIMB_SPEED):
                    self.pyro_fire(now)
                    self.enter_state(STATE_DESCENT)
                elif tis >= MIN_COAST_TIME and self.confirmed(
                        self.baro_healthy and deploy_allowed and
                        (self.curr_speed < -APOGEE_SPEED_THRESHOLD or abs(self.curr_speed) < APOGEE_ABS_THRESHOLD or
                         self.curr_height < self.max_alt - APOGEE_ALT_DROP)):
                    self.pyro_fire(now)
                    self.enter_state(STATE_DESCENT)
            elif self.state == STATE_DESCENT:
                if self.baro_healthy and abs(self.curr_speed) < LANDING_SPEED_THRESHOLD:
                    if self.landing_since == 0:
                        self.landing_since = now
                    if now - self.landing_since >= LANDING_STABLE_TIME:
                        self.enter_state(STATE_LANDED)
                else:
                    self.landing_since = 0
            elif self.state == STATE_LANDED:
                if (now // 500) % 2:
                    b.buzzer.tone(3550)
                else:
                    b.buzzer.off()
            if self.pyro1_tick and now - self.pyro1_tick >= PYRO_ON_TIME:
                b.pyro.off(1)

            # --- SD log: 20 Hz, f_sync once a second ---
            if self.log_open and now - last_log >= LOG_PERIOD_MS:
                last_log = now
                self.log_line(now, imu_ok, deploy_allowed, baro_good, accel, accel_mag)
            if self.log_open and now - last_sync >= SD_SYNC_PERIOD_MS:
                last_sync = now
                b.sd.sync()
            if not self.log_open and self.state not in (STATE_BOOST, STATE_COAST) and \
                    now - last_retry >= LOG_RETRY_PERIOD_MS:
                last_retry = now
                self.log_open = b.sd.mount() and self.log_open_new()
                if not self.log_open:
                    self.log_errors += 1

            # --- IMU watchdog ---
            if now - last_diag >= DIAG_PERIOD_MS:
                last_diag = now
                if imu_age > 500:
                    b.imu.begin()

            # --- Radio telemetry: sample at 8 Hz, one frame of 8 samples per second ---
            if now - last_tm >= TM_SAMPLE_PERIOD_MS:
                last_tm += TM_SAMPLE_PERIOD_MS
                if now - last_tm >= TM_SAMPLE_PERIOD_MS:
                    self.tm_send_frame()
                    last_tm = now
                self.tm_take_sample(last_tm, accel)
                if len(self.tm_samples) >= TM_SAMPLES_PER_FRAME:
                    self.tm_send_frame()

            self.loop_work_ms = b.millis() - now

    def pyro_fire(self, now):
        self.pyro1_tick = now or 1
        self.b.pyro.fire(1)


def main(board):
    Flight(board).run()
