# Tube Furnace Firmware

PlatformIO firmware for the tube furnace temperature controller, targeting the
**ESP-WROOM-32**. It reads a K-type thermocouple (MAX6675), runs a PID loop, and
time-proportions a solid-state relay (SSR) to drive the heating element. Live
telemetry is streamed over WebSockets (JSON) and the serial port (CSV).

## Project layout

```
Firmware/
├── platformio.ini                 # board, framework, dependencies
├── include/
│   ├── config.h                   # pin map, PID gains, control timing, ramp program
│   ├── secrets.example.h          # WiFi credentials template
│   └── secrets.h                  # your real credentials (gitignored)
├── src/
│   └── main.cpp                   # application: wires the libs together
├── lib/                           # self-contained, reusable libraries
│   ├── PIDController/             # generic SISO PID (anti-windup, output limits)
│   ├── MAX6675/                   # documented K-type thermocouple driver
│   ├── TelemetryServer/           # WebSocket broadcast server
│   └── TimeProportionalActuator/  # non-blocking slow-PWM SSR driver
└── tools/
    └── plotter/                   # Python live serial plotter (dev tool)
```

Each library under `lib/` is independent and carries a `library.json`, so it can
be copied into another PlatformIO project (or published) on its own. None of
them know anything about the furnace — `PIDController`, for example, works for
any sensor/actuator loop.

## First-time setup

1. Install [PlatformIO](https://platformio.org/) (CLI or the VS Code extension).
2. Create your credentials file:
   ```sh
   cp include/secrets.example.h include/secrets.h
   # edit include/secrets.h with your WIFI_SSID / WIFI_PASSWORD
   ```
3. Adjust `include/config.h` for your hardware (pins) and process (PID gains,
   setpoint program).

## Build, flash, monitor

```sh
pio run                 # build
pio run -t upload       # flash over USB
pio device monitor      # serial monitor @ 115200
```

## Wiring (default `config.h` pin map)

| Signal            | ESP32 GPIO | Notes                              |
|-------------------|-----------:|------------------------------------|
| MAX6675 SCK       | 18         | VSPI clock                         |
| MAX6675 SO (MISO) | 19         | data out of the sensor             |
| MAX6675 /CS       | 5          | chip select                        |
| SSR gate          | 13         | active-high, drives the heater     |

The MAX6675 is read-only, so no MOSI line is wired.

## Telemetry

* **WebSocket** (port 81): one JSON frame per control window, e.g.
  `{"temperature":42.25,"setpoint":50.00,"duty_cycle":0.3200,"tc_connected":true,"uptime_ms":12345}`
* **Serial** (115200): CSV `Temp,Setpoint,Duty`, consumed by `tools/plotter`.

## Live plot (development)

The Python plotter reads the serial CSV stream:

```sh
cd tools/plotter
uv sync
uv run plotter.py      # set PORT inside plotter.py to your serial device
```

## Design notes

* **Non-blocking control loop.** The SSR is time-proportioned by
  `TimeProportionalActuator` using `millis()` comparisons rather than
  `delay()`, so the WebSocket server is serviced continuously instead of being
  starved for a full window.
* **Anti-windup PID.** `PIDController` uses conditional integration: the
  integrator stops accumulating while the output is saturated and the error
  would drive it further past the rail, avoiding post-step overshoot.
* **Fail-safe.** An open thermocouple (MAX6675 reports `NAN`) forces the heater
  off and flags `tc_connected:false` in telemetry.
