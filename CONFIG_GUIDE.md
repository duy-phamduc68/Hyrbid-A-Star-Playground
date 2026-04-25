# CONFIG_GUIDE.md

This document explains all configuration parameters in the Hybrid A* + kinematic + SAT-based collision pipeline.

---

# 1. planner

## `path_model`

* Options: `RS`, `Dubins`
* Controls analytic shortcut model used in search.
* Affects:

  * ability to generate smooth goal connections
  * cost of final connection (“sniper shot”)

---

# 2. environment

## `json_file`

* Scenario definition input (start, goal, obstacles)

## `width`, `height`

* World size in pixels

## `grid_size`

* Size of one grid cell (px)
* Used for:

  * discretizing position into bins
  * distance field resolution
  * collision grid lookup

## `yaw_resolution_deg`

* Discretization resolution for yaw bins
* Used in visited state hashing:

  ```python
  int(yaw // yaw_resolution_deg)
  ```

---

# 3. vehicle

## `length`, `width`

* Physical vehicle size (px scale)
* Used for:

  * corner computation
  * collision SAT checks
  * circumscribed/inscribed radius

## `wheel_base`

* Used in kinematic bicycle model:

  ```python
  yaw += (v / wheel_base) * tan(steer)
  ```

---

# 4. steering

## `multipliers`

* Defines discrete steering actions per expansion step:

  * `-1` → left
  * `0` → straight
  * `1` → right

* Combined with `steer_max_deg`:

  ```
  steer = multiplier * steer_max_deg
  ```

---

# 5. kinematics

## `v_max`

* Forward/backward speed magnitude per step

## `steer_max_deg`

* Maximum steering angle

## `dt`

* Time duration of ONE search expansion step

### Key meaning:

Each node expansion integrates motion over:

> `dt` seconds

---

# 6. animation

Controls visualization only (no effect on search correctness)

## `enable_search_animation`

* Show exploration process

## `delay_search`, `delay_snake`

* Frame delay

## `search_render_every`

* Render frequency during search

## `snake_render_every`

* Rendering frequency for final path

## `final_path_speed`

* Playback speed multiplier for final trajectory

---

# 7. smoothing (POST-PROCESS ONLY)

Runs after a valid path is found.

## Purpose

Refines geometry of the final path only.
Does NOT affect search.

---

## Parameters

### `iterations`

* Number of gradient descent iterations

---

### `data_weight`

* Pulls path toward original solution

---

### `smoothness_weight`

* Encourages straightness (Laplacian smoothing)

---

### `obstacle_weight`

* Pushes path away from obstacles using distance field

---

### `curvature_weight`

* Penalizes high curvature (non-physical turns)

---

### `safety_buffer`

* Distance threshold to obstacles for smoothing influence

---

### `max_step`

* Maximum per-iteration movement per point

---

# 8. search

## `h_cost_weight`

* Weight of heuristic in A*:

  ```
  f = g + h * weight
  ```

* Higher → more greedy search

* Lower → more optimal but slower

---

## `min_g_improvement`

State pruning threshold:

A node in the same grid cell is ignored if:

```
new_g >= best_g_in_cell - min_g_improvement
```

### Intuition:

Controls how aggressively duplicates are pruned.

* LOW value:

  * more nodes kept
  * slower
  * higher path diversity

* HIGH value:

  * more pruning
  * faster
  * risk of missing better paths

---

## `sniper_trigger_radius`

* Distance threshold to attempt analytic shortcut (RS/Dubins)

---

# 9. sampling

This defines how continuous motion becomes discrete evaluation.

---

## `kinematic_substeps`

Refines integration inside one expansion.

### What it does:

Splits one `dt` step into smaller integration steps.

Example:

```
dt = 0.1
kinematic_substeps = 3
→ integration step = 0.033s each
```

### Effect:

* smoother arcs
* better curvature approximation
* denser collision sampling

### DOES NOT affect:

* number of nodes
* branching factor
* search depth

---

## `analytic_step_px`

Used only for analytic (RS / Dubins) path discretization.

### Purpose:

Break continuous curve into points for collision checking.

### Effect:

* smaller → more accurate but slower
* larger → faster but may skip collisions

---

## `min_chunks`

Minimum segmentation of analytic paths.

Ensures:

* even short shortcuts are not under-sampled
* avoids single-step “teleport-like” checks

---

# 10. numeric

## `angle_wrap_deg`

* Angle normalization modulus

## `tiny_epsilon`

* Numerical stability threshold for comparisons

---

# 11. dilation (collision acceleration system)

This is a **multi-layer collision rejection pipeline**.

---

## `collision_radius` (inscribed circle)

* Fast reject
* If distance ≤ this → guaranteed collision

---

## `overlay_radius` (circumscribed circle)

* Fast accept
* If distance ≥ this → guaranteed safe (skip SAT)

---

## `margin`

* Grid discretization safety buffer
* Prevents missed collisions due to cell center approximation

---

# 12. penalties

Used only in cost function (g-cost)

## `reverse`

* Cost multiplier for reversing

## `gear_shift`

* Cost for switching direction

## `steer`

* Steering penalty (currently disabled in your config)

---

# 13. proximity

Soft cost shaping (NOT hard collision)

## `penalty_weight`

* Weight of distance-field penalty

## `wall_buffer`

* Distance influence zone for walls

## `car_buffer`

* Distance influence zone for dynamic/continuous obstacles

---

# 14. goal_tolerance

Defines success condition

## `distance`

* Position threshold to accept goal

## `yaw_deg`

* Orientation threshold

---

# Summary mental model

You can think of the system as 4 layers:

### 1. Motion generation

* kinematics
* steering
* sampling

### 2. Search logic

* A*
* heuristic
* pruning (`min_g_improvement`)

### 3. Collision system

* grid distance field
* SAT fallback
* dilation radii

### 4. Post-processing

* smoothing only (geometry refinement)
