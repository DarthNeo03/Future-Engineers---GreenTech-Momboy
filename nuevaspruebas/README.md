# nuevaspruebas — the car we race

This is the current system, and the folder is named after the branch it comes
from (`nuevas_pruebas`) so there is never a doubt about which program is which.
It runs **both** rounds: the Open Challenge and the Obstacle Challenge are the
same program with a different calibration profile, not two codebases.

```
nuevaspruebas/
├── raspberry-pi/      the pilot program (Python 3.11 + OpenCV)
│   ├── main.py        entry point
│   ├── piloto.sh      daemon: runs it with no web panel (for competition)
│   ├── src/           vision, geometry, walls, lines, navigation, obstacles, button, link
│   ├── config/        params.json and colors.json — profiles, three lines of them
│   ├── tools/         selftest.py — 323 checks, no hardware needed
│   └── README.md      the deep technical notes (Spanish, our working language)
└── firmware/esp32/    hardware controller: motor, servo, gyro, colour, button, failsafe
```

## Running it

```bash
cd raspberry-pi
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

python3 tools/selftest.py        # 323 checks, no camera, no car
python3 main.py                  # camera + ESP32 + web panel
python3 main.py --simulado       # on a laptop, without the ESP32
python3 main.py --imagen foto.jpg  # on a still, without a camera
python3 main.py --vmax 70        # PWM ceiling — always the first run of the day
```

ESP32: open `firmware/esp32/esp32_carro.ino` in the Arduino IDE (all files in
the same folder) and upload. No external libraries. **Upload the firmware before
running `main.py`** — if the protocol changed, old firmware just sits in
failsafe.

Then pick the profile line for the round you are practising: **Open Challenge**,
**Avoid obstacles**, or **Obstacles + parking**. They are stored separately, so
tuning one round can never damage another round's working calibration.

## Starting a round

One button on GPIO 13 — press to arm and start, press again to disarm and stop.
The blue LED on GPIO 2 tells you the state from across the track. In
competition, run it through `piloto.sh` so there is no web panel and no network
on the car at all.

## Open Challenge — three laps of an empty track

1. **The direction is worked out by the car.** Nothing may be fed to the robot
   before the start, so the direction comes from the first floor line it crosses
   — orange first is clockwise, blue first is counter-clockwise — with the camera
   as a second opinion and the side of the first turn as a third.
2. **It stays in the lane** by comparing free space left and right in
   millimetres, not pixels, with the gyro holding the heading on the straights.
3. **The corner is three moves:** brake and let the rear wheels clear the inner
   edge, turn 90° closed by the gyro, then straight again. The trigger we trust
   is the floor line, because it is a physical fact and not an inference.
4. **It counts corners, not laps.** A corner opens with its first line and waits
   for the other one to close it; four corners make a lap. If a line is missed,
   the completed 90° turn counts it instead — once, either way.
5. **It stops by itself** after corner twelve, driving on just long enough to sit
   entirely inside the finish section, which is where the bonus is.

### The two failures that cost us the most

**The shiny wall.** The first version looked for the lowest black pixel in each
column. The moment a wall caught the light it stopped being black, the contact
line jumped to the chairs behind the track, and the car believed the road was
clear. The fix was to stop asking "where is the black?" and start asking "where
does the floor stop being floor?", cutting everything above the geometric
horizon first.

**The corner loop.** When the inner wall ends it leaves an enormous gap of white
floor, and to any free-space navigator that gap *is* the road: the car drove into
it, saw another gap from its new position, and never came out. In one recorded
capture the corridor measured 1064 mm of clear road at exactly the moment the car
had to turn. No threshold fixes that. The answer was the floor line plus a
commitment: once inside a corner, the 90° turn runs to completion and the camera
cannot redirect it.

## Obstacle Challenge — the same laps, with traffic signs

Red pillars are passed on their right, green on their left, and passing one on
the wrong side ends the round. So the side comes from the **colour** and is never
negotiable; only the distance is clipped to the free gap. If the gap is too tight
for a comfortable margin the car squeezes to the physical minimum (half a car
plus half a pillar); if not even that fits, it says so on screen and wall
avoidance takes over.

Left and right are the *vehicle's*, so nothing needs flipping when the round runs
the other way. And because the cost of being wrong is the whole round, the
overlay draws a green arrow to the pass point and writes "red → pass on its
right", so we check it with the car standing still, before the round.

Three failures worth reading:

1. **Steering saturated as the car closed in** — the angle to the pass point was
   computed from the distance to the pillar, so the same lateral offset demanded
   more angle the closer it got: 96 % of lock at 600 mm, 100 % at 400 mm. Fixed
   with a minimum look-ahead, a ceiling of 55 % on what avoidance may ask for,
   and a slew-rate limit.
2. **The rear wheel swept the pillar away.** Up close the pillar leaves the
   camera's view, the avoidance vanished, lane-centring pulled the car back to
   the middle, and with Ackermann steering the tail cuts inside. Now there is an
   overtaking commitment: from the moment the pillar disappears, the car holds
   straight for as long as it needs to clear it with its whole length.
3. **A pillar against the inner wall drove the car into the corner.** The pillar
   could take 100 % of the command exactly when the wall was the real problem.
   Now its authority fades as the corridor narrows, and if the gap is narrower
   than the car, the car aims at the middle of the gap instead of picking a side.

One filter that is easy to miss: **pillars seen behind the floor lines belong to
the next section** and are ignored until the car enters the corner zone. Reacting
to them early drags the car onto the inner corner exactly when it should be
setting up to turn.

### Parking — the open item

The bay is 20 cm wide and 1.5 × the length of the robot, in the starting section.
The short chassis pays off twice here: a shorter car needs a shorter bay and
turns in a tighter circle to reach it.

> **Status:** the rear-camera slot is reserved (`camara.indice_trasera`) and the
> "Obstacles + parking" profile line exists, but the manoeuvre itself is still
> being written.
