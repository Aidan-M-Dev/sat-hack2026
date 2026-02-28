"""Python model of the car control logic from demo.ino.
This module mirrors the decision-making so it can be exercised with
unit tests (red/green cycle) and used as a reference when auditing the
Arduino sketch.

Originally the Arduino demo added a fixed `+20` bias to one motor to
"compensate for yaw"; that turned out to make some cars spin in tight
circles. The current pure‑python model reflects the corrected approach
– equal PWM on both motors except during explicit reverse/turn
commands.

The behaviour is intentionally simple: when an obstacle is closer than
"set_dis" the car stops, backs up, and turns left. Otherwise it moves
forward at a speed that slowly ramps to a ceiling.
"""

SET_DISTANCE = 10
MIN_SPEED = 120
MAX_SPEED = 235
REVERSE_SPEED = 140
TURN_SPEED = 140


class State:
    def __init__(self):
        self.speed_adjustment = MIN_SPEED
        self.stop_bit = 0


def step(distance: float, state: State):
    """Perform one control loop iteration.

    Returns a list of commands representing the motor actions taken.
    Each command is a tuple; for forward it is ("forward", speed), for
    others it is the string name ("stop", "reverse", "turn_left").

    The state object is modified in place.
    """
    commands = []
    if distance < SET_DISTANCE:
        commands.append("stop")
        commands.append("reverse")
        commands.append("turn_left")
        state.speed_adjustment = MIN_SPEED
        state.stop_bit = 1
    else:
        # ramp toward faster speed when there is no obstacle
        if state.speed_adjustment < MAX_SPEED:
            state.speed_adjustment += 1
        commands.append(("forward", state.speed_adjustment))
        state.stop_bit = 0
    return commands
