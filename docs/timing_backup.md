# pick_place.py — Original Timing Values (Backup)

Recorded before any speed adjustments. Revert to these if the simulation becomes unstable.

## COOLDOWN_SECS

```python
COOLDOWN_SECS = 1.0  # wait before the first phase begins
```

## PHASE_DURATIONS (seconds each phase is held)

```python
PHASE_DURATIONS = {
    "NEUTRAL":          5.0,
    "MOVE_ABOVE_CUBE":  5.0,
    "LOWER":            4.0,
    "GRASP":            3.0,
    "LIFT":             4.0,
    "MOVE_TO_TARGET":   5.0,
    "LOWER_TO_TARGET":  4.0,
    "RELEASE":          2.0,
    "LIFT_GRIPPER":     3.0,
    "LIFT_BEFORE_HOME": 4.0,
    "HOME":             6.0,
}
```

**Total sequence time (excluding cooldown):** 45.0 seconds
