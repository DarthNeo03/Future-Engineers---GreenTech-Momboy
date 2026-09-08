# Open Challenge

Everything the car needs for the Open Challenge: three laps of an empty track,
in the direction the judges pick after check time, and a stop inside the finish
section.

```
open-challenge/
├── raspberry-pi/      the pilot program (Python 3.11 + OpenCV)
│   ├── main.py        entry point
│   ├── src/           vision, geometry, walls, lines, navigation, race, link
│   ├── config/        params.json and colors.json (profiles)
│   ├── tools/         selftest.py — 295 checks, no hardware needed
│   └── README.md      the deep technical notes (in Spanish, our working language)
└── firmware/esp32/    the hardware controller (Arduino C++, no external libraries)
```

> The same program also drives the Obstacle Challenge. We keep a full copy in
> each folder so that either challenge can be read, cloned and run on its own;
> what changes between them is the calibration profile and which behaviours are
> switched on. If you fix something here, carry it across — the journal entry
> for 2 September explains why we accepted that duplication.

## Running it

```bash
cd raspberry-pi
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

python3 tools/selftest.py        # 295 checks, no camera, no car
python3 main.py                  # camera + ESP32 + web panel
python3 main.py --simulado       # on a laptop, without the ESP32
python3 main.py --imagen foto.jpg  # on a still, without a camera
python3 main.py --vmax 70        # PWM ceiling, first run of the day
```

The ESP32 side: open `firmware/esp32/esp32_carro.ino` in the Arduino IDE (all
five files in the same folder) and upload. **Upload the firmware before running
`main.py`** — if the protocol changed, old firmware just sits in failsafe.

Pick the **Open Challenge** profile line in the panel before calibrating, so
that tuning the obstacle round never touches the calibration that works here.

## What the car actually does

1. **It works out the direction by itself.** The rules forbid feeding the robot
   anything before the start, so the direction comes from the first floor line
   it crosses — orange first means clockwise, blue first means counter-clockwise
   — with the camera as a second opinion and the side of the first turn as a
   third.
2. **It stays in the lane** by comparing free space left and right in
   millimetres, not pixels, with the gyro holding the heading on the straights.
3. **It takes the corner in three moves:** brake and let the rear wheels clear
   the inner edge, turn 90° closed by the gyro, then straight again. The trigger
   we trust most is the floor line, because it is a physical fact rather than an
   inference.
4. **It counts corners, not laps.** A corner opens with its first line and waits
   for the other one to close it; four corners make a lap. If a line is missed,
   the completed 90° turn counts it instead — it is counted once either way.
5. **It stops on its own** after corner twelve, driving on for a set time so the
   whole car sits inside the finish section, which is where the bonus is.

Three minutes is the cap, and the program enforces it.

## The two failures that cost us the most

**The shiny wall.** The first version looked for the lowest black pixel in each
column. The moment a wall caught the light it stopped being black, the contact
line jumped to the chairs behind the track, and the car believed it had a clear
road. The fix was to stop asking "where is the black?" and start asking "where
does the floor stop being floor?", cutting everything above the geometric
horizon first.

**The corner loop.** When the inner wall ends it leaves an enormous gap of white
floor. To any free-space navigator that gap *is* the road: the car drove into
it, saw another gap from the new position, drove into that, and never came out.
No threshold fixes this — the vision was answering the wrong question. The
answer was the floor line plus a commitment: once the car is inside a corner,
the 90° turn runs to completion and the camera cannot redirect it.

Both are written up in detail in `raspberry-pi/README.md`.
