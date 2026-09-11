# main — the car we race

This is the current system: the program that races, kept in `main/` so there is
never a doubt about which of the two is which (the one before it lives in
[`../fallback/`](../fallback/)).

**One folder per round.** The two challenges used to be one program with two
calibration profiles. They are now two programs, because sharing them meant
every experiment for the Obstacle Challenge could break a clean Open Challenge
run the night before a competition, and in the pits the only thing you want to
change is which `main.py` you launch. The Open Challenge folder is the version
that reliably completes three laps, kept deliberately small; the Obstacle
Challenge folder started as a copy of it and adds exactly two things.

```
main/
├── open-challenge/     three laps of an empty track. No pillars, no red, no green
├── obstacle-challenge/ the same, plus pillar avoidance and the corner rescue
│   ├── main.py         entry point (both folders have the same shape)
│   ├── piloto.sh       daemon: runs it with no web panel (for competition)
│   ├── src/            vision, geometry, walls, lines, navigation, button, link
│   ├── config/         params.json and colors.json — one calibration per round
│   ├── tools/          selftest.py — no hardware needed
│   └── README.md       the deep technical notes (Spanish, our working language)
├── raspberry-pi/       the previous combined program, kept until the two above
│                       have been through a full track session
└── firmware/esp32_carro/  hardware controller: motor, servo, gyro, colour, button,
                        failsafe. SHARED — the same binary serves both rounds
```

## Running it

```bash
cd open-challenge          # or: cd obstacle-challenge
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

python3 tools/selftest.py        # no camera, no car
python3 main.py                  # camera + ESP32 + web panel
python3 main.py --simulado       # on a laptop, without the ESP32
python3 main.py --imagen foto.jpg  # on a still, without a camera
python3 main.py --vmax 70        # PWM ceiling — always the first run of the day
```

The two folders are independent on purpose: each has its own `config/`, so
tuning obstacle avoidance cannot touch the calibration that just completed three
clean laps.

ESP32: open `firmware/esp32_carro/esp32_carro.ino` in the Arduino IDE (all files in
the same folder) and upload. No external libraries. **Upload the firmware before
running `main.py`** — if the protocol changed, old firmware just sits in
failsafe.

**Flash the firmware before the first run of the day.** The colour sensor now
integrates for 12 ms instead of 24 (ATIME 251) and the ESP32 polls it at 200 Hz
instead of once per integration period — polling at the same rate as the chip
integrates meant the two drifted and a sample was lost every other cycle, so
24 ms of integration produced up to 48 ms between readings and a 2 cm line fitted
in the gap without leaving a single reading. Three readings per line now, where
before there were sometimes none.

After flashing, **re-sample white on the Calibrate tab**: halving the integration
time halves every raw count, so the light floor (`tcs.c_min`) moves. The
thresholds that actually decide the colour are ratios and the difference between
them, and those do not change — which is exactly why they are the discriminators.

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

### The corner rescue — when the floor becomes a triangle

The failure that the Obstacle Challenge folder exists to fix: the car enters a
corner and does not come out. Black fills nearly the whole frame and the floor
that is left is a **triangle with its vertex at the top** — the two walls of the
corner closing in from both sides. Reversing does not help; it opens a hand's
width of corridor, hands control back to lane-centring, and lane-centring drives
straight back into the same gap.

It is detected on the wall profile that is already being computed, with no extra
vision: wall in nearly every column, nothing far away even at the 80th
percentile, a profile with **relief** (a flat wall ahead gives 0 % and is a
different problem, solved by the existing reverse), little floor left, and all of
that held for `rescate.confirmar_ms` without the vertex moving away. Then the car
reverses briefly with opposite lock and commits to a turn of about 100° toward
the inside of the lap — or toward the side a visible pillar demands, because
passing one on the wrong side ends the round while being stuck only costs time.

If the rescue ever fires in a good corner, raise `confirmar_ms` before touching
anything else.

Two measurements worth keeping: the relief threshold is a **percentage, not
millimetres** — the same 90° corner gives 27 % from 700 mm and 31 % from 400, but
in millimetres it drops from 147 to 89, so a millimetre threshold would fail
exactly when the car is most stuck. And the way out of a corner is read from
**how much floor is visible on each half of the image**, not from which side is
further away: wedged into a corner both sides are equally close and the distance
comparison says nothing, while the white pixels still do.

### Parking — the open item

The bay is 20 cm wide and 1.5 × the length of the robot, in the starting section.
The short chassis pays off twice here: a shorter car needs a shorter bay and
turns in a tighter circle to reach it.

> **Status:** the rear-camera slot is reserved (`camara.indice_trasera`) and the
> "Obstacles + parking" profile line exists, but the manoeuvre itself is still
> being written.
