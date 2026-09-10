# Power budget

> **Status:** the topology below is what the car is built around. The numbers in
> the "typical" column are datasheet / vendor figures, **not** our measurements,
> and the battery row is still to be filled in by the team. We are keeping the
> two apart on purpose: a documentation table that mixes what you measured with
> what you copied is worth nothing when something browns out at the venue.

## Rails

```
BATTERY  ──▶ main switch ──┬──▶ IBT-2 (BTS7960)  B+/B-   ──▶ DC motor
                           │
                           ├──▶ regulator 5 V  ──┬──▶ Raspberry Pi 5 (USB-C)
                           │                     │        └─▶ camera (USB)
                           │                     │        └─▶ ESP32 (USB)
                           │                     │
                           │                     └──▶ MG996R servo (5–6 V)
                           │
                           └──▶ (common ground with everything above)
```

Two things we learned the hard way and that this drawing has to keep true:

- **The motor and the sensors must not share a soft rail.** On an early
  prototype the ESP32 probed for the MPU-6050 and the TCS34725 right after
  `Wire.begin()`, while the motor start-up was pulling the rail down, and both
  sensors came back as "absent" *for the rest of the run*. We fixed it in
  firmware (wait 250 ms, retry every 3 s, plus a manual retry button in the
  panel), but the real fix is electrical: keep the logic rail stiff.
- **The servo is the spiky load, not the motor.** An MG996R under a steering
  correction pulls hard for a few tens of milliseconds. If it sags the 5 V rail
  the Pi is the one that reboots, and losing the Pi mid-round means losing the
  round.

## Consumers

| Load | Voltage | Typical (datasheet / vendor) | Measured on our car |
|---|---|---|---|
| Raspberry Pi 5 (8 GB), OpenCV running | 5 V | 3–5 A peak | **TODO** |
| USB camera WN-L1812.K56R | 5 V (from Pi) | ~0.2 A | **TODO** |
| ESP32 dev board | 5 V (from Pi USB) | 0.15–0.5 A on peaks | **TODO** |
| MPU-6050 + TCS34725 | 3.3 V (from ESP32) | ~0.02 A together | **TODO** |
| Servo MG996R | 5–6 V | ~0.5–0.9 A moving, ~2.5 A stalled | **TODO** |
| DC motor through the IBT-2 | battery | depends on the surface | **TODO** |

> **TODO (team):** battery pack — chemistry, cell count, nominal voltage,
> capacity, BMS, and how long a full charge lasts in practice (that last one is
> the number that decides how many practice rounds we get between charges).
> Also: main switch model and position, and whether there is a fuse.

## How we plan to measure it

1. Bench first, with the wheels in the air: `python3 main.py --vmax 70`, a USB
   meter on the Pi and a clamp meter on the battery lead.
2. Then a full three-lap round with the same instruments, logging the worst
   instantaneous draw and the resting voltage right after the round.
3. Repeat at the end of a practice session, with the pack half empty — brownouts
   show up at low state of charge, not at full charge.
