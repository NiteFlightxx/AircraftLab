# AircraftAutopilot architecture

## Responsibility

AircraftAutopilot executes one movement intent and publishes a continuous,
physically constrained setpoint to `UFlightControllerComponent`.

It does not perform navigation, obstacle avoidance, perception, gameplay
decision making, mission sequencing, arming or weapon control.

External systems own target and collision-free path generation:

```text
Navigation / gameplay system
    -> FAutopilotMovementIntent
    -> UAutopilotComponent
    -> TrajectoryGenerator
    -> PathFollowing (path and orbit intents only)
    -> TurnBehavior
    -> MotionProfile
    -> FeedForward
    -> FAutopilotInjection
    -> UFlightControllerComponent
```

## Movement intents

`FAutopilotMovementIntent` is the only public movement command.

- `Hold`: retain the position and heading captured when submitted.
- `MoveToPosition`: fly to a fixed position or an actor-relative position.
- `MoveWithVelocity`: continuously track a world-space velocity.
- `FollowPath`: follow collision-free world-space points supplied externally.
- `Orbit`: continuously orbit a fixed position or actor-relative position.

FollowPath supports two trajectory modes:

- `PiecewiseLinear`: exact polyline following with the existing trapezoidal timing.
- `MinimumSnap`: a native-time, piecewise seventh-order trajectory with globally
  minimized squared snap, waypoint P/V/A/Jerk continuity and iterative time
  scaling for speed, acceleration and jerk limits.

MinimumSnap accepts at most 16 path points in one synchronous solve. Longer
paths must be submitted in overlapping sections or moved to a future
background/sparse solver. It may cut corners between waypoints, so navigation
must only select it when the resulting curve is contained by a safe corridor.

Each accepted intent receives a unique `FAutopilotIntentHandle`. The same
handle can update the current intent without resetting MotionProfile state.
Submitting a new intent interrupts the previous handle. Cancelling or
finishing an intent enters an internal hold so an active Autopilot never loses
its setpoint.

## Fixed tick order

Every `TG_PrePhysics` tick performs exactly this sequence:

1. Capture one immutable vehicle snapshot from FlightController.
2. Resolve moving target changes and rebuild trajectory only when required.
3. Produce the nominal setpoint for the active intent.
4. Apply path guidance for FollowPath and Orbit.
5. Resolve heading mode.
6. Compute coordinated-turn feed-forward.
7. Apply MotionProfile velocity, acceleration, jerk and yaw limits.
8. Update hover-thrust estimation and feed-forward.
9. Evaluate completion and publish one cached injection.

FlightController pulls that cache through `IAutopilotProvider` later in the
same frame. The component registers itself as FlightController's tick
prerequisite.

## Configuration

`UAutopilotProfileAsset` is the authored configuration source. It contains:

- MotionProfile limits;
- feed-forward parameters;
- coordinated-turn limits and enable flag;
- path-guidance strategy and enable flag;
- hover-thrust estimator parameters and enable flag.

Intent constraints can only reduce the profile's speed and acceleration
limits. They cannot raise the vehicle capability declared by the profile.

## External navigation contract

Navigation provides world-space targets or path points. AircraftAutopilot
does not alter a path to avoid obstacles and does not infer obstacle state.
When navigation replans, it calls `UpdateMovementIntent` with the active
handle. A changed path rebuilds trajectory geometry while MotionProfile keeps
its current position, velocity and acceleration state.

## Completion contract

Finite position and path intents use separate horizontal and vertical
tolerances, speed tolerance, yaw tolerance and stable time. Pass-through
intents complete without stopping and preserve their exit velocity until a new
intent arrives. Hold, velocity and orbit intents run until cancelled,
interrupted or timed out.

## Tests

Automation tests under `Private/Tests` cover:

- asymmetric acceleration and deceleration;
- finite trajectory completion timing;
- terminal speed and short-path feasibility;
- infinite orbit trajectory and guidance continuity;
- MotionProfile transition continuity;
- yaw-rate acceleration limiting.
