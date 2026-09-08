# main — the previous system, kept runnable

This folder is the code that lives on the `main` branch: **reconizer**, the
system we drove before the current pilot. It is here on purpose, and it is not
decoration — it still starts, still drives, and it is what we fall back to if
something goes badly wrong with the new program during a practice session.

```
main/
├── raspberry-pi/      reconizer: the earlier pilot (Python + OpenCV)
│   ├── main.py        entry point
│   ├── src/           vision, navigation, obstacles, laps, link
│   ├── config/        colors.json and robot.json
│   └── tools/         calibrator, desktop panel, self-tests
└── firmware/esp32/    the SAME firmware as nuevaspruebas/ (see below)
```

## Running it

```bash
cd raspberry-pi
pip install -r requirements.txt
python3 main.py --vmax 70
```

Switching between the two systems is just which `main.py` you launch. That was
the whole reason for splitting the repository by branch instead of by round.

## The firmware is shared — read this before flashing

`main/firmware/esp32/` and `nuevaspruebas/firmware/esp32/` hold the **same**
firmware, the one with the physical button and the status LED. One car, one
microcontroller, one binary to flash: keeping two firmwares and remembering
which one is on the ESP32 today is exactly the kind of mistake that costs a
round.

What that means in practice:

- **The button works here too.** The arm/disarm button on GPIO 13 and the cut it
  latches live inside the ESP32, so with this firmware the physical button stops
  the car even though reconizer's Python knows nothing about it.
- **The Python side of the button is not ported.** `botones.py` and the protocol
  changes that read the button bits exist only in `nuevaspruebas/`. Reconizer is
  code we no longer develop, and writing an untested port of it into a program
  we do not race would be worse than saying this plainly.
- **The link still works** because the command frame did not change: same sync
  bytes, same CRC, same six-byte command. The newer firmware sends a sensor frame
  (yaw, floor colour, button) that reconizer's older protocol never asked for and
  simply does not read.

> **Check on the bench, not on the track:** wheels in the air,
> `python3 main.py --vmax 70`, confirm the car responds to the panel, then press
> the button with the car armed and confirm the motor cuts. Do that once after
> flashing and this whole section stops being a worry.

## What it taught us

Most of the failure list in [`../journal/engineering-journal.md`](../journal/engineering-journal.md)
was learned on this program: the wall measured as "the lowest black pixel",
which fails the moment a wall shines; the vote accumulator that saturated in one
second and always answered "clockwise"; the 180° turn measured with a short
angular difference, which wraps and never finishes.

Its handover document — the full list of mistakes we had already made by August,
written the day they happened — is at
[`../journal/prototypes/CONTEXTO-reconizer.md`](../journal/prototypes/CONTEXTO-reconizer.md).
It is in Spanish, our working language.
