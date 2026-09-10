# Images

| Folder | What goes in it |
|---|---|
| `team/` | Photos of the team (one serious, one not so serious) |
| `vehicle/` | The car from every side, from the top and from the bottom — required |
| `build/` | Build and workshop photos: printed parts, wiring, the car on the track |
| `test-captures/` | Frames the car itself saved during test runs, and the diagnostics we drew on them |
| `open.gif` | The car running the Open Challenge |

`test-captures/` is not decoration. Those sequences are the evidence behind
half the decisions in the journal: the shiny-wall failure, the corridor that
measured 1064 mm of clear road exactly where the car had to turn, the blue line
the colour sensor kept missing. Anyone can replay them with
`main/raspberry-pi/main.py --imagen <file>` and see what the car saw.
