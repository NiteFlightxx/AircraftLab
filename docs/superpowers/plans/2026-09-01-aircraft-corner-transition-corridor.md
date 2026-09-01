# Aircraft Analytic Capsule Corridor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace plane-based straight/corner corridor cells with one analytic capsule per canonical navigation leg, using the shared capsule end-cap volume for rounded turns and exact route ownership boundaries.

**Architecture:** `AircraftRuntimeInterface` owns the reflected capsule data and exact geometry operations. `AircraftSafeCorridorBuilder` emits a rounded route plus one capsule per canonical input leg, with each corner midpoint acting as the exact route ownership boundary. `AircraftSpatialPath`, MPCC-facing correction, diagnostics, and drawing consume the same capsule geometry; no polyhedral or separate-sphere representation remains.

**Tech Stack:** Unreal Engine 5.7 C++, USTRUCT reflection, `FVector`, analytic segment/capsule geometry, AircraftLab Autopilot and Diagnostics modules.

**Spec:** `docs/superpowers/specs/2026-09-01-aircraft-corner-transition-corridor-design.md`

## Global Constraints

- Preserve user-owned `.uasset` and `.umap` changes.
- Do not retain `BoundaryPlanes`, polygonal prisms, corner hulls, separate corner spheres, `CrossSectionSides`, deprecated fields, overloads, or fallback builders.
- `StoredRadiusCm = OuterRadiusCm`; apply `CorridorSafetyMarginCm` exactly once when evaluating the effective capsule.
- Do not change physical speed planning, flight-control, drive-mode, LOD, or networking semantics.
- Do not run automated tests. Update test sources only where the public data-model change requires compilation, then verify with static checks and Development/DebugGame Editor builds.

---

### Task 1: Replace Plane Corridor Data With Analytic Capsule Data

**Files:**
- Modify: `Source/AircraftRuntimeInterface/Public/AircraftRuntimeInterface/AircraftMovementIntent.h`
- Modify: `Source/AircraftRuntimeInterface/Private/AircraftMovementIntent.cpp`

**Interfaces:**
- Produces capsule fields `AxisStartCm`, `AxisEndCm`, and `RadiusCm` on `FAircraftSafeCorridorSegment`.
- Produces `IsGeometryValid()`, `GetClosestAxisPoint(...)`, `ComputeCorrectionCm(...)`, and `ComputeRayExitParameter(...)`.
- Preserves `ResolveAircraftSafeCorridorSegment(...)` and its half-open Route interval contract.

- [ ] **Step 1: Replace `BoundaryPlanes` with reflected capsule fields and focused methods**

```cpp
UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Navigation", meta = (Units = "cm"))
FVector AxisStartCm = FVector::ZeroVector;

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Navigation", meta = (Units = "cm"))
FVector AxisEndCm = FVector::ZeroVector;

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Navigation", meta = (ClampMin = "0.0", Units = "cm"))
float RadiusCm = 0.0f;

bool IsGeometryValid() const;
FVector GetClosestAxisPoint(const FVector& PositionCm) const;
FVector ComputeCorrectionCm(const FVector& PositionCm, float SafetyMarginCm) const;
bool ComputeRayExitParameter(const FVector& StartCm, const FVector& DirectionCm,
    float SafetyMarginCm, float& OutExitParameter) const;
```

- [ ] **Step 2: Implement exact capsule membership and correction**

Use the clamped projection parameter on `[AxisStartCm, AxisEndCm]`. Return zero correction when squared radial distance is no larger than `(RadiusCm - SafetyMarginCm)^2`; otherwise return the vector from the point to the nearest point on the effective capsule surface.

- [ ] **Step 3: Implement analytic ray exit**

Transform the ray into axial/radial components, solve the quadratic intersection with the cylindrical side and both endpoint spheres, accept only roots on the corresponding side/hemisphere, and return the unique nonnegative exit parameter for a start point inside the effective capsule. Zero direction returns an unbounded parameter; invalid geometry, invalid margin, non-finite values, or an outside start returns false.

- [ ] **Step 4: Replace Route validation**

Validate finite nonzero capsule axes, finite positive radii, finite positive contiguous Route intervals, first start zero, and last end equal to Route length. Remove all plane validation.

- [ ] **Step 5: Static check**

Run: `git diff --check -- Source/AircraftRuntimeInterface`

Expected: no whitespace errors and no `BoundaryPlanes` reference in RuntimeInterface.

### Task 2: Emit One Capsule Per Canonical Navigation Leg

**Files:**
- Modify: `Source/AircraftAutopilot/Public/AircraftAutopilot/AircraftSafeCorridorBuilder.h`
- Modify: `Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSafeCorridorBuilder.cpp`

**Interfaces:**
- Consumes `FAircraftPathOptimizationRuntimeConfig::CorridorSafetyMarginCm` and `ResampleSpacingCm`.
- Produces a rounded `FAircraftRouteIntent` with exactly `CanonicalPoints.Num() - 1` capsule cells.

- [ ] **Step 1: Remove polygon configuration and geometry**

Delete `CrossSectionSides` from `FAircraftSafeCorridorBuildSettings`, delete all prism/plane/hull helpers, and update the public comment to describe analytic capsules.

- [ ] **Step 2: Keep strict canonical path construction**

Reject non-finite points, remove consecutive points shorter than `MinimumSegmentLengthCm`, remove forward-collinear internal points, and return `DegenerateTurn` for exact reverse turns.

- [ ] **Step 3: Calculate turn extents from the effective radius**

```cpp
const float EffectiveRadiusCm = Settings.OuterRadiusCm - PathConfig.CorridorSafetyMarginCm;
```

Initialize each internal corner's entry and exit extent to this value. For each source leg, proportionally scale the start exit and end entry extents if their sum exceeds the leg length.

- [ ] **Step 4: Emit exact rounded-route structure**

For each corner emit a straight span to `Entry`, sample the quadratic Bézier separately over `[0, 0.5]` and `[0.5, 1]`, and always append the exact midpoint `B(0.5)`. Record its cumulative Route distance as the ownership boundary between the incoming and outgoing capsules.

- [ ] **Step 5: Build the capsule partition after the route**

For canonical leg `i`, assign:

```cpp
Segment.AxisStartCm = Points[i].PositionCm;
Segment.AxisEndCm = Points[i + 1].PositionCm;
Segment.RadiusCm = Settings.OuterRadiusCm;
Segment.StartDistanceCm = i == 0 ? 0.0f : CornerBoundaryDistancesCm[i - 1];
Segment.EndDistanceCm = i + 1 == LegCount ? RouteLengthCm : CornerBoundaryDistancesCm[i];
```

Require exactly one positive Route interval per leg and validate the final strict partition before returning `Succeeded`.

- [ ] **Step 6: Static check**

Run: `rg -n "CrossSectionSides|BoundaryPlanes|BuildCornerPlanes|AddPrismPlanes" Source/AircraftAutopilot/Public/AircraftAutopilot/AircraftSafeCorridorBuilder.h Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSafeCorridorBuilder.cpp`

Expected: no matches.

### Task 3: Make Spatial Path Preserve Capsule Boundaries and Use Capsule Constraints

**Files:**
- Modify: `Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSpatialPath.cpp`

**Interfaces:**
- Consumes capsule methods from Task 1.
- Preserves explicit source Route distance mapping on every Quintic segment.

- [ ] **Step 1: Make resampling boundary-aware**

For every input polyline span, merge its uniform sample distances with all Corridor `StartDistanceCm`/`EndDistanceCm` values strictly inside that span, sort and deduplicate, then interpolate positions using Route distance. Exact Corridor boundary distances must survive filtering and resampling.

- [ ] **Step 2: Preserve ownership knots during optimization**

If a Knot's incoming and outgoing sample segments resolve to different capsule indices, keep the Knot at its constructed Route position. Otherwise optimize normally and apply the selected capsule's exact `ComputeCorrectionCm` once.

- [ ] **Step 3: Replace plane control-hull scaling**

For each equivalent Quintic Bézier control delta, call `ComputeRayExitParameter(...)` on the required capsule and accumulate the minimum scale. Apply one common scale to the Knot's first and second derivatives to retain C2 continuity.

- [ ] **Step 4: Replace plane sample validation and runtime correction**

Use `ComputeCorrectionCm(...).Size()` for full-curve validation, `ComputeCorridorViolationCm`, and `ComputeCorridorCorrectionCm`. Report `CorridorCapsuleViolation` with the capsule index and violation distance; remove plane iteration and plane-index diagnostics.

- [ ] **Step 5: Static check**

Run: `rg -n "BoundaryPlanes|CorridorPlaneViolation|InvalidCorridorPlane" Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSpatialPath.cpp`

Expected: no matches.

### Task 4: Align Diagnostics, Public Blueprint Data, and Compile-Time Fixtures

**Files:**
- Modify: `Source/AircraftDiagnostics/Public/AircraftDiagnostics/AircraftDebug.h`
- Modify: `Source/AircraftDiagnostics/Private/AircraftDebug.cpp`
- Modify: `Source/AircraftDiagnostics/Public/AircraftDiagnostics/AircraftDebugDraw.h`
- Modify: `Source/AircraftDiagnostics/Private/AircraftDebugDraw.cpp`
- Modify: `Source/AircraftDiagnostics/Private/AircraftDebugAutopilotOptions.cpp`
- Modify only for compilation: `Source/AircraftAutopilot/Private/Tests/AircraftAutopilotTests.cpp`

**Interfaces:**
- Produces `FAircraftDebugDraw::DrawCapsule(...)`.
- Preserves green current, red actual violation, orange predicted violation, and cyan-blue inactive colors.

- [ ] **Step 1: Replace plane-specific failure diagnostics**

Remove `CorridorPlaneIndex`; add capsule axis start/end, stored radius, and effective radius fields. Update the single planning-failure log format accordingly.

- [ ] **Step 2: Add authoritative capsule drawing**

Add `DrawCapsule(Context, AxisStartCm, AxisEndCm, RadiusCm, Color, Segments, Thickness)`. For PDI use `DrawWireCapsule` with half-height `0.5 * AxisLength + Radius`; for World use `DrawDebugCapsule` with local Z rotated onto the capsule axis.

- [ ] **Step 3: Replace polyhedron reconstruction**

Delete plane intersection, vertex, and edge reconstruction from `AircraftDebugAutopilotOptions.cpp`. Draw each stored capsule directly through `FAircraftDebugDraw::DrawCapsule` while preserving the existing dynamic color selection.

- [ ] **Step 4: Update compile-time test fixtures without running tests**

Replace manually assigned `BoundaryPlanes` with `AxisStartCm`, `AxisEndCm`, and `RadiusCm`; replace direct plane containment assertions with `ComputeCorrectionCm(...).IsNearlyZero()` or a positive correction assertion.

- [ ] **Step 5: Static check**

Run: `rg -n "CrossSectionSides|BoundaryPlanes|CorridorPlaneIndex" Source`

Expected: no matches.

### Task 5: Review and Compile

**Files:**
- Review all files changed by Tasks 1-4.
- Do not modify user assets.

- [ ] **Step 1: Inspect source-only changes**

Run: `git diff --check`

Run: `git diff -- Source/AircraftRuntimeInterface Source/AircraftAutopilot Source/AircraftDiagnostics`

Expected: only capsule-corridor work plus the user's already-restored related source changes; no asset staging or rewriting.

- [ ] **Step 2: Compile Development Editor**

Run:

```powershell
& 'E:\UnrealEngine\UnrealEngine_Source\Engine\Build\BatchFiles\Build.bat' GASP57Editor Win64 Development '-Project=E:\UnrealProjects\GASP57\GASP57.uproject' -WaitMutex -NoHotReloadFromIDE
```

Expected: `Result: Succeeded`.

- [ ] **Step 3: Compile DebugGame Editor**

Run:

```powershell
& 'E:\UnrealEngine\UnrealEngine_Source\Engine\Build\BatchFiles\Build.bat' GASP57Editor Win64 DebugGame '-Project=E:\UnrealProjects\GASP57\GASP57.uproject' -WaitMutex -NoHotReloadFromIDE
```

Expected: `Result: Succeeded`.

- [ ] **Step 4: Report the manual scene checks**

Ask the user to verify: a straight two-point route; a 90-degree turn; adjacent turns on a short middle leg; current/actual/predicted corridor colors; and an exact reverse route returning `DegenerateTurn`.
