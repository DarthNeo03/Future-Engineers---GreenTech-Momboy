# 3D model

Every structural part of the car is printed. The chassis is not a box with
things screwed onto it: the motor mount, the gear supports, the steering rack
housing, the camera mast and the electronics deck are features of the same
model, which is the only way we got the car down to the size it is now.

```
3d-model/
├── stl/              parts as printed
├── print-projects/   sliced projects (.3mf) with the settings we actually used
└── README.md
```

## Current design

**WRO CAR XVII** — the seventeenth iteration of the chassis, and the one the car
runs today.

| | |
|---|---|
| Overall size | ≈ 200 mm long × 180 mm wide × 150 mm tall |
| Rules limit | 300 × 200 mm footprint, 300 mm height, 1500 g |
| Wheelbase | ≈ 150 mm |
| Wheel diameter | 65.2 mm |
| Drive | one DC motor → 2:1 gearing → rear axle with a mechanical differential |
| Steering | MG996R on a rack and pinion, Ackermann geometry, ≈26° at full lock |
| Camera | 125 mm above the floor, tilted 7.5° down |
| Material | PLA |

> **TODO (team):** weigh the finished car and write the number here. It is the
> one figure in this table we have not measured, and the limit is 1500 g.

## Why it got smaller instead of prettier

The first chassis was built to hold the parts. This one is built to fit the
track, and every millimetre we removed bought something specific:

- **The parking bay is 20 cm wide and 1.5 × the car's length.** A longer car
  needs a longer bay and has to hit it more precisely. Shortening the car is the
  cheapest possible improvement to the parking manoeuvre — it happens before any
  code runs.
- **Turning circle.** With a 150 mm wheelbase and ~26° of lock the car turns
  inside roughly 310 mm of radius. That is what lets it take a corner inside the
  narrow (600 mm) track configuration without a three-point turn.
- **The tail sweeps less.** With Ackermann steering the rear wheel cuts inside
  the front one. A shorter, narrower car sweeps a smaller area, and that is
  exactly the area that used to knock over the pillar we had just passed.
- **Weight and the print queue.** Smaller parts print faster and fail less. When
  a part broke two days before a test we could reprint it the same afternoon
  instead of losing the week.
- **Everything the camera needs is fixed by geometry.** Camera height and tilt
  are model dimensions, not adjustments — the software converts pixels to
  millimetres from those two numbers, so a chassis that holds the camera rigidly
  at 125 mm and 7.5° is worth more than any amount of calibration.

## Reprinting it

The `.3mf` projects carry the settings we used: 0.4 mm nozzle, 0.2 mm layers,
PLA. Print the base flat with no supports; the gear supports and the steering
parts want more perimeters than infill — they take load, and infill does not
carry it.

> **TODO (team):** export the parts of **WRO CAR XVII Final** to `stl/` and to
> STEP, and replace the older exports currently in this folder (they are from
> the June–July iterations). The Fusion source file is ~200 MB, well over
> GitHub's 100 MB per-file limit, so it does not belong in the repository: put a
> Fusion shared link here instead, and keep STL + STEP + 3MF as the files people
> can actually use.
