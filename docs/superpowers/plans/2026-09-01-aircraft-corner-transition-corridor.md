# Aircraft Corner Transition Corridor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace per-leg-only corridor construction with a strictly safe straight/corner convex-cell partition and a rounded reference route that can traverse turns without artificial near-zero speed.

**Architecture:** `AircraftRuntimeInterface` owns the single corridor-interval resolver and strict route contract. `AircraftSafeCorridorBuilder` canonicalizes the navigation polyline, builds straight prisms and waypoint-centered inscribed corner bipyramids, allocates short-leg transition extents, and emits a rounded reference route with matching Route intervals. `AircraftSpatialPath` and `AircraftDiagnostics` consume the shared resolver; the physical timing planner remains unchanged.

**Tech Stack:** Unreal Engine 5.7 C++, USTRUCT/UENUM reflection, `FVector`, `FPlane`, `TArray`, AircraftLab Dataflow runtime.

**Spec:** `docs/superpowers/specs/2026-09-01-aircraft-corner-transition-corridor-design.md`

## Global Constraints

- `OuterRadiusCm` remains a hard Euclidean clearance radius.
- `CorridorSafetyMarginCm` is read from RuntimeConfig and is not duplicated as a builder setting.
- Keep the existing Blueprint entry point and three builder settings.
- Do not add old-builder compatibility, minimum corner speed, tolerance relaxation, or fallback paths.
- Do not modify `AircraftMotionPlan` physical speed-limit semantics.
- Preserve user-owned assets and unrelated source changes.
- Per user instruction, do not run automated tests; verify with compiler builds and provide a manual scenario checklist.

---

### Task 1: Centralize Corridor Route-Interval Semantics

**Files:**
- Modify: `Source/AircraftRuntimeInterface/Public/AircraftRuntimeInterface/AircraftMovementIntent.h`
- Modify: `Source/AircraftRuntimeInterface/Private/AircraftMovementIntent.cpp`

**Interfaces:**
- Produces: `int32 ResolveAircraftSafeCorridorSegment(TConstArrayView<FAircraftSafeCorridorSegment> Corridor, float RouteDistanceCm, float RouteLengthCm)`.
- Produces: strict contiguous Corridor validation used by every Route intent.

- [ ] **Step 1: Declare the exported resolver after `FAircraftSafeCorridorSegment`**

```cpp
AIRCRAFTRUNTIMEINTERFACE_API int32 ResolveAircraftSafeCorridorSegment(
    TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
    float RouteDistanceCm,
    float RouteLengthCm);
```

- [ ] **Step 2: Implement one ordered half-open lookup**

```cpp
int32 ResolveAircraftSafeCorridorSegment(
    const TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
    const float RouteDistanceCm,
    const float RouteLengthCm)
{
    if (Corridor.IsEmpty() || !FMath::IsFinite(RouteDistanceCm)
        || !FMath::IsFinite(RouteLengthCm) || RouteLengthCm <= UE_SMALL_NUMBER
        || RouteDistanceCm < -UE_KINDA_SMALL_NUMBER
        || RouteDistanceCm > RouteLengthCm + UE_KINDA_SMALL_NUMBER)
    {
        return INDEX_NONE;
    }
    const float DistanceCm = FMath::Clamp(RouteDistanceCm, 0.0f, RouteLengthCm);
    for (int32 Index = 0; Index < Corridor.Num(); ++Index)
    {
        const FAircraftSafeCorridorSegment& Segment = Corridor[Index];
        const bool bLast = Index == Corridor.Num() - 1;
        if (DistanceCm >= Segment.StartDistanceCm
            && (DistanceCm < Segment.EndDistanceCm
                || (bLast && DistanceCm <= Segment.EndDistanceCm)))
        {
            return Index;
        }
    }
    return INDEX_NONE;
}
```

- [ ] **Step 3: Tighten Route validation**

Require a non-empty Corridor to begin at zero, contain only positive-length finite cells, share the exact previous end value within `UE_KINDA_SMALL_NUMBER`, contain finite nonzero planes, and end at the computed Route length within `UE_KINDA_SMALL_NUMBER`. Remove separate overlap/gap compatibility clauses.

- [ ] **Step 4: Inspect the diff**

Run: `git diff --check -- Source/AircraftRuntimeInterface`

Expected: no whitespace errors.

### Task 2: Build Straight and Corner Convex Cells

**Files:**
- Modify: `Source/AircraftAutopilot/Public/AircraftAutopilot/AircraftSafeCorridorBuilder.h`
- Modify: `Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSafeCorridorBuilder.cpp`

**Interfaces:**
- Consumes: `FAircraftSafeCorridorBuildSettings`, `FAircraftPathOptimizationRuntimeConfig`.
- Produces: `EAircraftSafeCorridorBuildStatus::DegenerateTurn`.
- Produces: a canonical rounded `FAircraftRouteIntent` whose cells strictly partition its Route length.

- [ ] **Step 1: Add the explicit degenerate-turn status**

```cpp
DegenerateTurn UMETA(DisplayName = "Degenerate Turn"),
```

- [ ] **Step 2: Replace the builder-local geometry with focused helpers**

Create private helpers with these responsibilities and signatures:

```cpp
bool IsFiniteVector(const FVector& Value);
void AddPrismPlanes(const FVector& Start, const FVector& End,
    float EndCapExtensionCm, float CrossSectionApothemCm,
    int32 CrossSectionSides, TArray<FPlane>& OutPlanes);
bool BuildCornerPlanes(const FVector& Center, const FVector& EntryRay,
    const FVector& ExitRay, float RadiusCm, int32 CrossSectionSides,
    TArray<FPlane>& OutPlanes);
float ComputeInsetRayExtent(const FVector& Center, const FVector& Ray,
    TConstArrayView<FPlane> Planes, float SafetyMarginCm);
FVector EvaluateQuadraticBezier(const FVector& Entry, const FVector& Control,
    const FVector& Exit, float Alpha);
```

`BuildCornerPlanes` must construct the N ring vertices plus two poles, orient every triangular face outward by comparing its normal with the corner center, and normalize every stored plane. It must not generate a circumscribed polyhedron.

- [ ] **Step 3: Canonicalize the input polyline**

Track original input indices while filtering invalid/short points. Remove forward-collinear internal points using the normalized direction cross/dot rule. Return `DegenerateTurn` with the responsible `InputPointIndex` for an exact reverse turn.

- [ ] **Step 4: Compute corner geometry and desired extents**

For each internal canonical point, store its boundary planes plus inset entry/exit ray extents. Reject non-finite or non-positive inset extents as `InsufficientClearance`.

- [ ] **Step 5: Fit adjacent corner extents to every source leg**

For each leg of length `L`, take the outgoing extent of its start corner and incoming extent of its end corner. If their sum exceeds `L`, multiply both by `L / Sum`. Endpoints use zero extent. This must leave a zero-or-positive straight remainder with no uncovered distance.

- [ ] **Step 6: Emit the rounded route and exact cell intervals in one pass**

Use a single route-point append helper that rejects only duplicate consecutive samples and updates cumulative distance. Emit:

```text
straight start -> corner entry -> sampled quadratic Bézier -> corner exit -> next straight
```

For every emitted straight or corner span, record the cumulative distance before and after the span and use those same values for `StartDistanceCm` and `EndDistanceCm`. Omit zero-length straight cells. Sample each Bézier with at least one interior point and maximum chord spacing no greater than `PathConfig.ResampleSpacingCm`.

- [ ] **Step 7: Validate the builder output before success**

Before returning `Succeeded`, require at least two output points, at least one positive-length cell, first start zero, exact adjacent interval continuity, and final end equal to cumulative Route length. Failure caused by available clearance returns `InsufficientClearance`; no old-builder fallback is allowed.

- [ ] **Step 8: Inspect the diff**

Run: `git diff --check -- Source/AircraftAutopilot/Public/AircraftAutopilot/AircraftSafeCorridorBuilder.h Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSafeCorridorBuilder.cpp`

Expected: no whitespace errors.

### Task 3: Make Spatial Path Use the Unique Corridor Cell

**Files:**
- Modify: `Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSpatialPath.cpp`

**Interfaces:**
- Consumes: `ResolveAircraftSafeCorridorSegment(...)` from Task 1.
- Preserves: explicit `RouteStartDistanceCm`/`RouteEndDistanceCm` mapping.

- [ ] **Step 1: Delete the local `FindCorridorSegment`**

Remove the anonymous-namespace predicate that treats both ends as inclusive.

- [ ] **Step 2: Constrain each optimized Knot to one cell**

Resolve the Knot's stored Route distance once, then project the candidate only against that cell's boundary planes:

```cpp
const int32 CorridorIndex = ResolveAircraftSafeCorridorSegment(
    Corridor, RouteDistancesCm[Index], RouteLengthCm);
if (CorridorIndex != INDEX_NONE)
{
    ProjectCandidateIntoPlanes(Candidate, Corridor[CorridorIndex].BoundaryPlanes,
        Config.CorridorSafetyMarginCm);
}
```

Pass `RouteLengthCm` into `OptimizeKnots`; do not recover it from optimized path arc length.

- [ ] **Step 3: Replace validation and runtime correction lookups**

Use the shared resolver for full-spline Corridor validation, `ComputeCorridorViolationCm`, and `ComputeCorridorCorrectionCm`. A non-empty Corridor that cannot resolve a sampled Route distance is a planning failure, not an unconstrained sample.

- [ ] **Step 4: Preserve explicit source-route mapping**

Keep `FSegment::RouteStartDistanceCm`, `RouteEndDistanceCm`, `GetRouteDistanceCm`, motion-plan sample Route distance, and trajectory Route progress unchanged.

- [ ] **Step 5: Inspect the diff**

Run: `git diff --check -- Source/AircraftAutopilot/Private/AircraftAutopilot/AircraftSpatialPath.cpp`

Expected: no whitespace errors and no `FindCorridorSegment` definition remains.

### Task 4: Align Corridor Debug Selection and Polyhedron Drawing

**Files:**
- Modify: `Source/AircraftDiagnostics/Private/AircraftDebugAutopilotOptions.cpp`

**Interfaces:**
- Consumes: `ResolveAircraftSafeCorridorSegment(...)` from Task 1.
- Preserves: green current, red actual violation, orange predicted violation, cyan-blue inactive colors.

- [ ] **Step 1: Use the shared resolver for the current segment**

```cpp
const int32 CurrentSegmentIndex = ResolveAircraftSafeCorridorSegment(
    Route.Corridor, CurrentRouteDistanceCm, RouteLengthCm);
```

Delete the local inclusive `IndexOfByPredicate` selection.

- [ ] **Step 2: Draw stored convex cells without synthesizing route caps**

`FAircraftSafeCorridorSegment::BoundaryPlanes` now completely bounds both prisms and corner bipyramids. Update `DrawCorridorSegment` to reconstruct vertices and edges from those stored planes directly. Do not add `StartPosition`/`EndPosition` cap planes, because doing so would clip the formal corner transition cell.

- [ ] **Step 3: Remove helpers made obsolete by cap synthesis**

Keep route sampling only where it still serves trajectory rendering. Remove unused `SampleCorridorPath` inputs from `DrawCorridorSegment` and any now-dead tangent calculations.

- [ ] **Step 4: Inspect the diff**

Run: `git diff --check -- Source/AircraftDiagnostics/Private/AircraftDebugAutopilotOptions.cpp`

Expected: no whitespace errors; all four existing color branches remain.

### Task 5: Static Review and Compiler Verification

**Files:**
- Review: all files modified in Tasks 1-4
- Do not modify: user-owned `.uasset` and `.umap` files

**Interfaces:**
- Verifies: module boundaries, Unreal reflection, generated headers, and both editor build configurations.

- [ ] **Step 1: Search for obsolete and duplicated semantics**

Run:

```powershell
rg -n "FindCorridorSegment|CorridorDistanceScale|IndexOfByPredicate" Source/AircraftAutopilot Source/AircraftDiagnostics Source/AircraftRuntimeInterface
```

Expected: no obsolete Corridor lookup remains; unrelated `IndexOfByPredicate` uses are reviewed individually.

- [ ] **Step 2: Review the final source diff**

Run: `git diff --check`

Expected: no whitespace errors.

Run: `git diff -- Source/AircraftAutopilot Source/AircraftRuntimeInterface Source/AircraftDiagnostics`

Expected: only approved corridor architecture and already-restored related work are present; no assets are touched.

- [ ] **Step 3: Compile Development Editor**

Run:

```powershell
& 'E:\UnrealEngine\UnrealEngine_Source\Engine\Build\BatchFiles\Build.bat' GASP57Editor Win64 Development '-Project=E:\UnrealProjects\GASP57\GASP57.uproject' -WaitMutex -NoHotReloadFromIDE
```

Expected: `Result: Succeeded`.

- [ ] **Step 4: Compile DebugGame Editor**

Run:

```powershell
& 'E:\UnrealEngine\UnrealEngine_Source\Engine\Build\BatchFiles\Build.bat' GASP57Editor Win64 DebugGame '-Project=E:\UnrealProjects\GASP57\GASP57.uproject' -WaitMutex -NoHotReloadFromIDE
```

Expected: `Result: Succeeded`.

- [ ] **Step 5: Provide the manual scene checklist**

Report these user-run checks without executing automation:

1. Straight route: only one straight corridor and unchanged cruise behavior.
2. Two-leg 90-degree route: one visible corner cell, no uncovered gap, smooth nonzero turn speed where dynamics allow.
3. Short leg between two corners: transition cells meet without negative or missing straight cells.
4. Actual and predicted violations: current cell changes green/red/orange consistently; inactive cells remain cyan-blue.
5. Exact reverse route: builder returns `DegenerateTurn` instead of choosing an arbitrary side.
