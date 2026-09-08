# Obstacle Challenge

Same three laps, now with red and green traffic signs on the track and a parking
lot to finish in. Red pillars are passed on their right, green on their left, and
passing one on the wrong side ends the round — which is why almost everything in
this folder is about **not being clever**.

```
obstacle-challenge/
├── raspberry-pi/      the pilot program (Python 3.11 + OpenCV)
│   ├── main.py        entry point
│   ├── src/           obstaculos.py is the one that matters here
│   ├── config/        params.json and colors.json (profiles)
│   ├── tools/         selftest.py — 295 checks, no hardware needed
│   └── README.md      the deep technical notes (in Spanish, our working language)
└── firmware/esp32/    the hardware controller (Arduino C++, no external libraries)
```

Run it exactly like the Open Challenge (see `../open-challenge/README.md`), then
switch on `obstaculos.activo` in the panel and select one of the two obstacle
profile lines: **Avoid obstacles** or **Obstacles + parking**. Keeping the
profiles apart is deliberate: tuning this round must never disturb the Open
Challenge calibration that already works.

## How a pillar is passed

The colour decides the side, and the side is not negotiable. The pass point is
computed in real millimetres and then clipped to the free gap — but only in
distance, never across the pillar. If the gap is too tight for a comfortable
margin we squeeze down to the physical minimum (half the car plus half the
pillar); if not even that fits, the car says so on screen and the wall-avoidance
takes over. That rule comes straight from a bug that would have ended a round:
the clipping used to be able to move the target to the *other* side of a red
pillar, and the car would have passed it on the left.

Left and right are the **vehicle's** left and right, so nothing has to be
flipped when the round runs counter-clockwise. Because a mistake here is fatal,
the video overlay draws a green arrow to the pass point and writes "red → pass
on its right", so we check it with the car standing still, before the round.

## The three things that were wrong, and what fixed them

1. **Steering went to full lock.** The angle to the pass point was computed from
   the distance to the pillar, so as the car closed in, the same lateral offset
   demanded more and more angle — 96 % of steering at 600 mm, 100 % at 400 mm.
   Fixed with a minimum look-ahead, a ceiling on what avoidance may ask for
   (55 %), and a slew-rate limit so no single frame can throw the wheel.
2. **The rear wheel swept the pillar away.** Up close the pillar drops out of
   the camera's view, the avoidance vanished, the lane-centring pulled the car
   back to the middle, and with Ackermann steering the tail cuts inside. Now
   there is an **overtaking commitment**: from the moment the pillar disappears,
   the car holds straight for as long as it needs to clear it with its whole
   length, computed from real speed and the car's length.
3. **A pillar against the inner wall drove the car into the corner.** The pillar
   used to be able to take 100 % of the command exactly when the wall was the
   real problem. Now its weight fades as the corridor narrows, and if the gap
   between pillar and wall is narrower than the car, the car aims at the middle
   of the gap instead of choosing a side and wedging itself in.

There is one more filter that is easy to miss: **pillars behind the floor lines
belong to the next section**, not to this straight. Reacting to them from here
drags the car towards the inner corner exactly when it should be setting up for
the turn. The filter lifts the moment the car enters the corner zone, which is
when those pillars really do matter.

## Parking — the open item

The parking lot is 20 cm wide and 1.5 × the length of the robot, and it sits in
the starting section. This is where the short chassis pays off twice: a shorter
car means a shorter bay to hit, and a tighter turning circle to get into it.

> **Status:** the rear camera slot is reserved in the configuration
> (`camara.indice_trasera`) and the "Obstacles + parking" profile line exists,
> but the parking manoeuvre itself is the piece we are still working on. It is
> the top item in the journal's "what's next".
