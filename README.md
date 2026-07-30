# Robot— MATLAB Strategy

A competitive robot strategy written in MATLAB for a two-robot tournament on a 10×10
board. Developed as a project for the Signals and Systems course at Ruppin Academic
Center.

## The Game

Two robots compete on a bounded board scattered with fuel tanks and mines. Fuel is life:
every move burns it, mines destroy on contact, and the robot with more fuel wins when
the two close within 5 units. The strategy function is called once per turn and returns
a movement vector.

## Core Idea

**Fuel economy first, attack only when the math guarantees it.**

The strategy never attacks on impulse. It attacks only when a calculation — based on the
fuel burn rate it measures in real time — shows it will still hold the advantage at the
moment of contact.

## Strategy Components

### Smart fuel tank selection
Rather than always running to the nearest tank, every tank is scored:

```
score = myDist − 0.3 × opDist
```

A tank the opponent will reach first scores badly, so the robot doesn't waste fuel
losing a race it was never going to win.

### Opponent prediction and interception
If the opponent is closer to a fuel tank than we are, we assume they're heading for it,
compute their direction of travel, and project 3 steps ahead. In attack mode we aim at
that future intercept point rather than their current position — a shorter chase, less
fuel burned, and the opponent arrives at the meeting with less fuel than they would have
otherwise.

### Attack conditions
Attack triggers on either:
- A fuel advantage of 70+, *and* a projected advantage above 20 after accounting for the
  fuel that will burn on the way — using the burn rate measured from actual play, not an
  assumption.
- A local opportunity: opponent within 2.0 units with a 25+ advantage. Close and end it.

### Multi-mine avoidance
Line–circle intersection is checked against every active mine, not just the nearest. The
most dangerous mine is handled first, the path steers around it with a safety margin, and
the check repeats up to 3 times. The destination is clamped inside the board bounds so no
wall-collision penalties are incurred.

### Survival behaviours
- **Stuck detection** — each turn the robot compares its position to the previous one.
  Moving less than the threshold suggests it's trapped between mines, so it evaluates
  eight escape directions and takes the one furthest from any mine.
- **Corner retreat** — when outmatched, it retreats to the corner closest to itself and
  furthest from the opponent.
- **Endgame stalemate** — no fuel left on the map and behind on fuel? It stops completely.
  Standing still burns the minimum, forcing the opponent to spend their own fuel on every
  approach attempt.

## Tunable Parameters

| Parameter | Value | Meaning |
|---|---|---|
| `ATTACK_FUEL_ADVANTAGE` | 70 | Fuel lead required to enter attack mode |
| `FUEL_CRITICAL` | 50 | Emergency — refuel immediately, ignore everything else |
| `FUEL_COMFORTABLE` | 80 | Refuel when convenient |
| `CHASE_MIN_TURNS` | 6 | Minimum turns to commit to a chase |
| `INTERCEPT_LOOKAHEAD` | 3 | Steps ahead to predict the opponent |
| `MINE_SAFE_MARGIN` | 0.4 | Clearance beyond the mine radius |
| `STUCK_THRESHOLD` | 0.05 | Movement below this counts as stuck |

## Files

| File | Description |
|---|---|
| `MyRobotStrategy2.m` | The strategy function — game logic and helper functions |
| `matlab_project.pptx` | Project presentation (Hebrew) |

## Usage

```matlab
[move, mem] = MyRobotStrategy2(env, mem);
```

`env` carries the game state (positions, fuel, mines, fuel tanks, board limits). `mem`
is a persistent struct the function maintains across turns to track chase state, fuel
burn rate, and stuck detection. Pass an empty `mem` on the first turn.

Requires the tournament game engine to run.

## Helper Functions

`getDist` · `minDistToMines` · `findSafestCorner` · `checkLineCircle` — all nested inside
the main function file.
