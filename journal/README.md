# Journal

How this car was actually built, in the order it happened.

| | |
|---|---|
| [`engineering-journal.md`](engineering-journal.md) | The log: dated entries, decisions, and every failure that taught us something |
| `test-logs/runs/` | CSV logs the car wrote during the 29–30 August test sessions |
| `test-logs/config-agosto.json` | The configuration those runs were made with |
| `prototypes/CONTEXTO-reconizer.md` | The handover document of the previous system: every mistake we had already made by August |
| `prototypes/metodos_navegacion.html` | The comparison of five navigation methods that decided the current one |
| `prototypes/rpi-tools-agosto/` | The simulator and analysis tools from the same period |

## Why the old system is still here

The code of the previous system is not in this folder — it is in
[`../main/`](../main/), still runnable, because that is where the branch it comes
from put it. What lives here are its documents.

They are in the repository on purpose: they are the reason we know that measuring
the wall by "the lowest black pixel" fails on a shiny wall, that a vote
accumulator with a ceiling saturates in one second, and that a 180° turn measured
by short angular difference never finishes. `CONTEXTO-reconizer.md` is the full
list of mistakes we had already made by August, written the days they happened —
in Spanish, which is the language we work in.

Deleting all of it would have made the repository look cleaner and made the work
look smaller than it was.

## How we keep the journal

One rule: **write it the day it happens.** The dates in the journal match the
commits and the CAD files, because they were written next to them. Anything
reconstructed a month later would be a story, not a record.
