# MK-1A Flight Software: Latency Task

MK-1A is a single-board flight computer for a sounding rocket. It reads an IMU and a barometer,
steers two fins, decides when to fire the parachute, logs to an SD card, keeps flight state in
flash and sends telemetry to a ground station over a LoRa radio.

`flightsw/controller.py` is a direct Python port of the production C firmware. It flies, but
bench runs show trouble:

- the 100 Hz control loop misses its deadline, sometimes by hundreds of milliseconds
- the fins react to stale attitude data, and the vehicle wobbles more than it should
- the SD log has gaps, and data is lost when power is cut at the end of the flight

**Your job:** find out what is costing the time, then restructure the flight software so the
control path meets its deadline and reacts to fresh data, without losing logging or telemetry.
How you do that is up to you.

## Running

Requires Python 3.10+ and the standard library only.

```
python run.py              # one ~40 s bench mission, prints the bench report
python run.py --seed 3     # a different bench flight
python run.py --json       # full report as JSON
```

The SD card image from the last run is written to `out/`.

## Rules

1. **Only edit files in `flightsw/` other than `board.py`.** You can add new modules. `board.py`
   is the hardware. Don't read, patch, monkey-patch or reach into it: no `_private` attributes,
   no replacing its methods. Your submission is run against a fresh copy of the hardware.
2. **Keep the flight logic.** Thresholds, filter constants, state-machine conditions, the fin
   control law, the log format (`LOG_HEADER` columns) and the telemetry frame format must stay as
   they are. You are changing *when and where* things run, not *what* they decide.
3. Pass the IMU sample's `stamp` to `fins.set()` for the sample each fin command was computed
   from. Fin latency is measured from that stamp.
4. When `board.running()` returns `False`, power is about to be cut. Anything still running must
   stop promptly. Any peripheral call after power-off raises `PowerLost`.
5. Treat the hardware like real hardware: you have no datasheet timings. Measure.

## Hardware API

All times are in the board clock (`board.millis()`), which starts at power-on.

| Call | Notes |
|---|---|
| `board.millis()`, `board.micros()` | HAL_GetTick |
| `board.delay_ms(ms)` | HAL_Delay |
| `board.running()` | False once the mission is over |
| `board.imu.begin()` | BNO085 on I2C3. Enables reports |
| `board.imu.service()` | Reads pending reports. Returns `ImuData(quat, accel, gyro, new, stamp)`, or `None` on a bus error |
| `board.imu.recover()` | I2C bus recovery after errors |
| `board.baro.begin()` / `board.baro.read()` | BMP388 on I2C1. Returns `BaroData(pressure, temperature, stamp)`, or `None` |
| `board.gps.readline(timeout_ms)` | One NMEA sentence (bytes) or `None` |
| `board.gps.read()` | All bytes received since the last call (non-blocking) |
| `board.radio.write(data)` | E22 LoRa modem on USART6 |
| `board.sd.mount()`, `open_new()`, `write(data)`, `sync()`, `close()` | SD card via FatFs |
| `board.nvm.save(record)`, `erase()`, `used()`, `capacity()` | Flight-state record store (records up to 32 bytes) |
| `board.fins.set(ccr3, ccr4, stamp)` | Fin servo compare values |
| `board.pyro.fire(ch)`, `off(ch)`, `state(ch)` | Pyro channels (only channel 1 is used) |
| `board.buzzer.tone(hz)`, `off()` | |
| `board.adc.vdda()` | Supply voltage |

Peripherals are what they are on the real board: each one sits on its own bus and is **not
re-entrant**. The board report counts any misuse.

## Evaluation

Your submission runs on the bench flight **and on several unseen flights and hardware conditions**
(think about what real hardware does on a bad day). Roughly in order of weight, we look at:

- control-loop timing: deadline misses, worst-case gap, jitter
- sensor-to-fin latency
- flight events: launch detection, parachute deployment timing relative to apogee, landing
- data delivered: SD log (as it is on the card after the power cut) and telemetry received on the ground
- robustness: no crashes, no bus misuse, clean shutdown
- memory: the flight computer has very little RAM, and peak allocation is measured

A safe flight comes first. A fast loop that loses the vehicle scores zero.

## Deliverables

1. Your modified `flightsw/` folder.
2. `DESIGN.md` (1 to 2 pages):
   - what was blocking, and how you found out (numbers, please)
   - your new architecture: what runs where, how data moves between parts, who owns each peripheral
   - how you bound memory and what you drop or keep when a consumer falls behind
   - what you would do differently on the real STM32 (RTOS tasks, DMA, interrupts, double buffers…)
3. Before/after output of `python run.py`.

Time budget: about 4 to 6 hours. A clear, correct, well-argued design beats a clever, fragile one.
