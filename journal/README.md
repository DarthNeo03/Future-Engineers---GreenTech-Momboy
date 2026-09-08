# Journal

How this car was actually built, in the order it happened.

| | |
|---|---|
| [`engineering-journal.md`](engineering-journal.md) | The log: dated entries, decisions, and every failure that taught us something |
| `test-logs/runs/` | CSV logs the car wrote during the 29–30 August test sessions |
| `test-logs/config-agosto.json` | The configuration those runs were made with |
| `prototypes/reconizer/` | The previous complete system (May–August). Kept, not deleted |
| `prototypes/rpi-tools-agosto/` | The simulator and analysis tools from the same period |

## Why the old prototype is still here

`prototypes/reconizer/` is the system that came before the one we race. It is
in the repository on purpose: it is the reason we know that measuring the wall
by "the lowest black pixel" fails on a shiny wall, that a vote accumulator with
a ceiling saturates in one second, and that a 180° turn measured by short
angular difference never finishes. Its handover document,
`prototypes/reconizer/docs/CONTEXTO.md`, contains the full list of mistakes we
had already made by August — in Spanish, which is the language we work in.

Deleting it would have made the repository look cleaner and made the work look
smaller than it was.

## How we keep the journal

One rule: **write it the day it happens.** The dates in the journal match the
commits and the CAD files, because they were written next to them. Anything
reconstructed a month later would be a story, not a record.
