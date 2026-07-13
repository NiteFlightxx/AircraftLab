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

Gameplay and Blueprint code should use the typed command facade. Each command
contains only fields meaningful to that action:

- `SubmitMoveTo` / `UpdateMoveTo`
- `SubmitFollowPath` / `UpdateFollowPath`
- `SubmitOrbit` / `UpdateOrbit`
- `SubmitCircleArc` / `UpdateCircleArc`
- `SubmitVelocity` / `UpdateVelocity`
- `SubmitHold`

`SubmitMovementIntent` and `UpdateMovementIntent` remain available as low-level
compatibility APIs. New gameplay code does not need to populate the union-like
`FAutopilotMovementIntent` directly.

Every typed command composes a separate `FAutopilotHeadingOptions`. Movement can
face its velocity, keep its current yaw, use a fixed yaw, face the movement
destination, or face an independent world position/Actor. An Actor heading
target is sampled every tick. `UpdateHeadingTarget` changes only heading policy,
preserving the active movement handle and trajectory.

The supported movement primitives are:

- `Hold`: retain the position and heading captured when submitted.
- `MoveToPosition`: fly to a fixed position or an actor-relative position.
- `MoveWithVelocity`: continuously track a world-space velocity.
- `FollowPath`: follow collision-free world-space points supplied externally.
- `Orbit`: continuously orbit a fixed position or actor-relative position.
- `CircleArc`: follow a finite horizontal arc around a fixed position or
  actor-relative position.

FollowPath supports three trajectory modes:

- `PiecewiseLinear`: exact polyline following with the existing trapezoidal timing.
- `MinimumSnap`: a native-time, piecewise seventh-order trajectory with globally
  minimized squared snap, waypoint P/V/A/Jerk continuity and iterative time
  scaling for speed, acceleration and jerk limits.
- `Bezier`: treat the supplied world-space points as Bezier control points.

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
4. Apply path guidance for FollowPath, CircleArc and Orbit.
5. Resolve heading mode.
6. Compute coordinated-turn feed-forward.
7. Apply MotionProfile velocity, acceleration, jerk and yaw limits.
8. Update hover-thrust estimation and feed-forward.
9. Evaluate completion and publish one cached injection.

FlightController pulls that cache through `IAutopilotProvider` later in the
same frame. The component registers itself as FlightController's tick
prerequisite.

## Configuration

`UAutopilotProfileAsset` contains algorithm tuning only:

- feed-forward gains;
- coordinated-turn limits and enable flag;
- path-guidance strategy and enable flag;
- hover-thrust estimator parameters and enable flag.

Every movement command carries its own `MotionConstraints`. These constraints
are the sole source used by TrajectoryGenerator and MotionProfile. The
FlightController profile remains the independent hard physical safety limit.
Mass, gravity and hover collective are read from FlightController at runtime
and are not duplicated in the Autopilot profile.

## External navigation contract

Navigation provides world-space targets or path points. AircraftAutopilot
does not alter a path to avoid obstacles and does not infer obstacle state.
When navigation replans, it calls `UpdateMovementIntent` with the active
handle. A changed path rebuilds trajectory geometry while MotionProfile keeps
its current position, velocity and acceleration state.

For new code, navigation should call `UpdateMoveTo` or `UpdateFollowPath` instead
of the low-level update API. Patrol sequencing, random/loop route selection,
wait times, perception, investigation, pursuit, attack windows, line-of-sight,
weapon firing, retreat and formation decisions belong to gameplay. The plugin
only executes the selected movement primitive and reports lifecycle/progress
through the intent handle, `OnIntentStarted`, and `OnIntentFinished`.

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
