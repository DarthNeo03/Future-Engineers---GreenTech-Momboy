# Engineering journal — GreenTech Momboy

WRO 2026, Future Engineers, Venezuela.

This is the log we kept while building the car. It is not a report written
afterwards to look tidy: the entries are dated by what the repository and the
CAD files say, and the failures are in here because they are the part that
taught us something. Where we were wrong, we left it written down.

Two of us build this car, and neither of us built it alone. José took the
obstacle side, Cristian took the Open Challenge, and both of us worked on the
chassis and on every prototype in between. What made that split work was that
we never owned a subsystem privately: the branches are shared, the panel shows
the same telemetry to both, and whoever was at the track that day drove the
decisions of that day.

---

## Phase 1 — Seeing colours (May)

**7 May.** Repository created. We started from the only thing we were sure of:
whatever else the car does, it has to tell red from green and orange from blue,
under whatever light the venue has.

**29 May.** First colour detection working, and the folder structure
reorganised twice in the same day because the first one was already annoying.
Two decisions from this week survived to the final car:

- **Colour is calibrated by clicking on the object in the image**, not by typing
  HSV numbers. And the pixel that is clicked is the one that rules: from the
  patch around it, only pixels that look like it survive. Before that, one click
  on the edge of a pillar mixed in the background and the range grew until the
  mask swallowed the white floor.
- **Five rotating profiles.** Light at home is not light at the venue. Instead
  of one calibration we keep the last five and pick one from the panel.

**2 June.** Vision refactored into modules and a distance test added. The
lesson that pushed this was simple: we could not tell whether a bad run was the
colour, the distance or the steering, because it was all one file.

**3 June.** `chassis v7` in Fusion. Seven versions before the car had ever
driven a lap, and it was already too big.

**5 June.** The floor lines get drawn on screen: the orange and blue lines at
each corner, and the track limits. At this point they were only a drawing. It
took us two more months to understand that those two lines were the most
reliable information the car would ever get.

---

## Phase 2 — Building the thing (June–July)

**16–29 June.** First printed chassis. Motor mount, gear supports, the base,
the covers. The slicer projects from those days are still in
`3d-model/print-projects/` — including a base that took 4 h 25 min to print,
which is a number that shapes your week when a part breaks.

We learned to print for repair, not for elegance: a broken part you can reprint
the same afternoon beats a beautiful one that costs a day.

**Late June — the differential.** The rules forbid a differential-wheeled base,
and they require the driving wheels to be driven together through gearing. We
built a mechanical differential and a single drive motor into the rear axle:
one motor, 2:1 gearing, both wheels turning off the same axle. Ackermann
steering on the front through a rack and pinion driven by the MG996R.

**14–29 July — the chassis stops being a box.** Four full CAD revisions in two
weeks (`WRO CAR`, `vIV`, `V_`, `VII`). This is where the car changed shape
rather than getting details: the electronics deck, the camera mast and the
motor mount became features of one model instead of parts bolted to a plate.

### Why we redesigned instead of improving what we had

The honest reason is that the first chassis was designed around the parts, and
the track does not care about our parts. Once we had driven a few laps we could
list what was actually costing us points, and every item on the list was
geometry:

- **The car did not fit the problem.** The parking bay is 20 cm wide and 1.5
  times the length of the robot. A long car needs a long bay and has to place
  itself in it more precisely. Shortening the car improves parking before a
  single line of code runs.
- **It turned too wide.** In the narrow (600 mm) track configuration the old
  car needed almost the whole corridor to come round a corner. With a 150 mm
  wheelbase and ~26° of lock, the new one turns inside roughly 310 mm of radius.
- **The tail knocked over pillars we had already passed.** With Ackermann
  steering the rear wheel cuts inside the front one. A shorter, narrower car
  sweeps a smaller area — and that area was exactly where the pillar was.
- **The camera moved.** Every distance the software computes comes from two
  numbers: camera height (125 mm) and tilt (7.5°). On the old chassis the mast
  flexed, so those numbers were not constant, so no calibration was ever right.
  On the new one the mast is part of the structure.
- **Weight and print time.** Smaller parts print faster, fail less and leave
  margin under the 1500 g limit.

None of that would have been visible from a drawing. We had to build the big
one first.

---

## Phase 3 — Two brains (August)

**20 August.** ESP32 firmware in the repository. The architecture stops being
"the Pi does everything" and becomes two controllers with one job each: the Pi
sees and decides, the ESP32 moves and protects.

We took the Wi-Fi out of the ESP32 completely. Not because of the radio rule
alone — although the rules do forbid wireless while the vehicle runs — but
because two things that can give orders is one thing too many. The ESP32 obeys
binary frames over a cable, and if the Pi goes quiet for 300 ms it stops the car
by itself. That failsafe is the reason there is a microcontroller in the middle
at all.

**28–29 August.** Navigation rebuilt, obstacle avoidance implemented, and the
MPU-6050 and the TCS34725 finally reconnected and read from the ESP32.

**The sensors that "were not there".** The ESP32 probed the I2C bus in `setup()`,
right after `Wire.begin()` — before the sensors had woken up, and while the
motor start-up was pulling the rail down. It declared both absent, for the whole
run, every run. The fix is three lines and one habit: wait 250 ms, retry every
3 s while one is missing, and put a manual retry button in the panel.
The general lesson we wrote on the wall: *any hardware auto-detection needs an
automatic retry, a manual retry, and somewhere you can see whether it is there.*

**29–30 August.** Test runs with logging. The CSV files in `test-logs/runs/`
and the frame sequences in `../images/test-captures/` come from these sessions.
Being able to replay a run on a laptop, frame by frame, changed how we work:
from that week on, no fix was accepted because it sounded right, only because
the recorded run improved.

**30–31 August.** Wall, navigation and orientation corrections. Colour
calibration and obstacle parameters. Corner handling reworked.

---

## Phase 4 — The four bugs that were really one (September)

Everything in this phase came from the same root cause: **the car knew where it
was pointing, but not which straight it was on.**

**31 Aug – 2 Sept — the corner loop.** At the corner the car would circle
inside it, as if the gap were the road. And for any free-space navigator, it
*is*: when the inner wall ends it leaves an enormous gap of white floor, the car
drives into it, sees another gap from its new position, and repeats. In a real
capture the corridor measured 1064 mm of clear road at the exact moment the car
had to turn. No threshold fixes that — the vision was answering the wrong
question. Three pieces fixed it: the floor line triggers the corner by itself,
the 90° turn runs committed so the camera cannot redirect it, and a corner that
has been turned cannot trigger again.

**2 Sept — the phantom turn.** Crossing the orange line, the car turned
correctly; a while later the blue line arrived and triggered *another* 90° turn,
sometimes the wrong way and between pillars. The code treated each line
separately, so a late blue line looked like the first line of a new corner. Now
a corner **opens** with its first line and waits for the other to **close** it —
the second line can never open a new one. Tested against four corners in a row
losing the blue line every time: the count still comes out at four.

**2–3 Sept — the blue line the sensor never saw.** The TCS34725 counted oranges
and never a blue. Two causes, and the second was the one that bit: the stored
profile had a clear-channel minimum taken from a very bright white, but a
coloured line *absorbs* light and returns much less — the reading was thrown
away before the colour was even looked at. And the absolute thresholds had a
ridiculous margin: blue beat white by three units in its own channel. We
switched the discriminator to the difference between the blue and red channels
(+37 on blue against ~0 on white) and lowered the clear minimum. Sampling a line
now lowers that minimum automatically if it has to.

**5–6 Sept — the side of the pass, and the corner that ate the pillars.**
Passing a pillar on the wrong side ends the round, so this one got a rule of its
own: the side comes from the colour and is never negotiable; only the distance
is clipped. In the same days we added the colour-corner mode, where only one
colour counts and the turn **gives way the instant a pillar appears** —
because the forced 90° turn was knocking over the pillars sitting at the exit of
the corner.

**7 Sept — re-anchoring the heading.** The last one, and our favourite, because
the symptom made no sense: the car crossed the line, turned correctly, and then
kept turning the *other* way into the wall. The heading reference only advanced
90° when the code registered a corner. If the colour sensor missed the line, the
car took the corner physically — on white space alone — while the code still
believed the straight was the previous one. The gyro was then dutifully pulling
the car back towards a straight that no longer existed. Now, if the car has been
turned by a corner-sized angle for long enough, in the direction of the round,
it adopts the new straight: that was a corner nobody registered.

**8 Sept — the button.** Until this week the car was armed from the web panel,
which cannot be how a competition round starts: the rules want one action on a
robot already placed on the track, and no wireless while it runs. So the round
now starts with **one push button on GPIO 13**, working like a stopwatch — press
to arm and go, press again to stop — plus a status LED on GPIO 2.

Two decisions inside that are worth more than the button itself. First, the
press travels to the Pi as a **level, not a counter**: a counter would store
presses, and a serial glitch with an undelivered press could start the car by
itself on reconnect. With a level, what is lost is simply not executed. Second,
when the car is armed a press can only mean *stop*, so the **ESP32 latches the
cut locally** and the motor dies on the next 10 ms tick — no round trip over
serial, and it works even if the Pi is hung shouting "forward". The self-test
suite went from 295 to 323 checks with it.

**8 Sept — one repository instead of four branches.** Chassis **WRO CAR XVII**
closed as final, and the repository unified: the pilot and the button branch
merged, the documentation written, and the code split into `main/` (what we
race) and `fallback/` (the system before it), each runnable on its own. The point is being able to fall back to a
known-good program in the pits by changing which `main.py` we launch, instead of
reverse-engineering a git revert under pressure.

---

## Our coach

Our tutor and coach is **Msc. Egardo Paolini**.

The rules are explicit that a coach may not build or program the robot, and ours
did not. What our coach did do was harder to write in a commit: made sure a
track existed to test on, asked what we had *measured* every time we said
something "worked", made us stop and document at points where we would happily
have kept hacking, and kept two teenagers with very different working hours
turning up to the same sessions. The bug list in this journal exists because we
were asked, repeatedly, how we knew.

## What working as two people actually looked like

- **Split by challenge, not by layer.** José on obstacle detection, Cristian on
  the Open Challenge. It meant each of us owned a whole vertical slice — vision,
  decision and driving — for one round, and could debug it end to end without
  waiting for the other.
- **Branches, and merging them on purpose.** `main` for what works, working
  branches for what does not yet. The merge is where we reviewed each other, and
  it caught things a lone author never sees.
- **The panel is the shared language.** Everything is adjustable while the car
  runs, and everything is on screen. Two people who can both see the same
  telemetry argue about the car, not about opinions.
- **Everything testable runs without the car.** 323 checks that need no camera
  and no hardware. There is one car and two of us: whoever does not have it can
  still work.

## What we would tell the next team

1. **A physical fact beats a clever inference.** The floor line under the car
   fixed a problem that months of vision tuning did not.
2. **Instrument first, tune second.** Every fix in September came from a
   recorded run we could replay, not from reading code.
3. **Redesign the mechanics when the mechanics are the problem.** We spent
   weeks compensating in software for a chassis that was too long. The compact
   chassis solved it in one afternoon of CAD.
4. **A failure that is safe beats a failure that is silent.** When the wall is
   ambiguous, the code marks it "unknown" and ignores it, instead of guessing.
5. **Write it down the day it happens.** Everything above came out of notes and
   commit messages written the same day. Reconstructed a month later, none of it
   would have been true.

## What's next

- The parallel parking manoeuvre and the rear camera (the slot is reserved, the
  code is not written).
- Measure the power budget properly, and the finished car's weight.
- Take the required photos and record both videos.
