# Doxygen Rules for This Project (C++)

This project uses Doxygen style comments for public APIs. The goal is consistent, accurate docs with minimal noise.

## 1. Comment style

Use either style consistently, placed immediately above the symbol:

A. Line style (for less than three lines docs)
  /// ...

B. Block style (preferred for rest)
  /** ... */

Public API docs belong in headers. Avoid duplicating docs in `.cpp`.

## 2. Required content by importance

### Always required for public APIs
1. `@brief` one sentence: what the symbol does (observable behavior).
2. `@param` for every parameter (include units/frames/ranges; for vector/array/span/matrix-like inputs and outputs, always include expected size/shape and layout/stride when relevant).
   For pointers/spans/views, explicitly document nullability, how size is determined, and any lifetime/ownership rules.
3. `@return` for every non-`void` return value (include units/meaning and size/shape when applicable).

Use direction tags consistently:
  @param[in] x ...
  @param[out] y ...
  @param[in,out] z ...

### Parameter conventions (make these explicit)
Keep parameter docs in a consistent order when possible:
meaning → units → size/shape → ordering/frame → nullability/lifetime (if needed).

Use one of these size/shape forms:
- `size = n()` for 1‑D vectors (use a real API name when available: `n()`, `nq()`, `nv()`, etc).
- `length = <count_param>` for raw buffers.
- `shape = (rows, cols)` for matrices, and add `RowMajor`/`ColMajor` only if it matters.

Nullability/lifetime language:
- Optional pointer/view: say `may be nullptr` / `may be empty` and define behavior when omitted.
- Required pointer/view: add a real `@pre` (e.g., `ptr != nullptr`, `span.size() == n()`).
- If a pointer/view is stored, say so and state what must outlive what.
- If a function returns a reference/view to internal storage, document the lifetime/invalidation rules.

### Add when applicable
`@details` if the behavior/constraints are not obvious from the signature.

`@throws` only if it throws.

`@pre` only for real preconditions (caller must satisfy).  
`@post` only for real guarantees after success.

`@warning` only for misuse that can silently break behavior or safety.  
`@note` for clarifications that are not contracts.

`@see` / `@sa` to link related types or functions.

## 3. Usage examples and code blocks

Use a section:

  @par Usage
  @code{.cpp}
  ...
  @endcode

Include `Usage` only when the API is nontrivial or easy to misuse:
1. required call order or lifecycle
2. ownership or lifetime rules
3. threading assumptions
4. gotchas (frames, units, statefulness)

Do not add `Usage` for trivial getters/setters or obvious functions.

## 4. Brevity rules

1. If the name and signature already explain it, keep docs to `@brief` + required `@param`/`@return` only.
2. Do not restate types (avoid “param is a double”).
3. Keep `@brief` one line. Put longer text in `@details`.

## 5. Minimal templates

### 5.1 Simple function (no params, no return)
/// @brief Clears any active trajectory and switches to hold mode.
void clear_traj();

### 5.2 Function returning a cached reference (document lifetime)
/**
 * @brief Computes control effort in source joint order (matches actuator ordering).
 * @return Torque command [N·m], size = n() (source joint order).
 * @note The returned reference is valid until the next call to compute_ctrl_effort().
 */
const Vec& compute_ctrl_effort();

### 5.3 Function with a real precondition and possible throw
/**
 * @brief Configures the mapping between source joint order and model canonical order.
 * @param[in] src_names Joint names in source (I/O) order, size = n().
 * @param[in] prefix Optional joint name prefix to strip before matching.
 * @throws std::runtime_error If `src_names` is the wrong size, contains duplicates (after prefix stripping), or is missing any canonical joint.
 */
void configure_io(const std::vector<std::string>& src_names,
                  const std::string& prefix = "");

### 5.4 Optional pointer parameters (nullability + size + behavior)
/**
 * @brief Updates the robot state from inputs in source joint order.
 * @param[in] q_src Optional joint positions [rad], size = n(); may be nullptr (no update).
 * @param[in] qd_src Optional joint velocities [rad/s], size = n(); may be nullptr (no update).
 * @param[in] qdd_src Optional joint accelerations [rad/s^2], size = n(); may be nullptr (no update).
 * @param[in] t Optional sample time [s]; may be nullptr (no update).
 */
void set_joint_state(const Vec* q_src = nullptr, const Vec* qd_src = nullptr,
                     const Vec* qdd_src = nullptr, const double* t = nullptr);

### 5.5 Fixed-size vectors (spell out ordering/layout)
/**
 * @brief Sets base pose and updates gravity accordingly.
 * @param[in] pose_wb_xyzwxyz World-from-base pose, size = 7 [x y z qw qx qy qz].
 * @throws std::runtime_error If `pose_wb_xyzwxyz.size() != 7`.
 */
void set_base_pose_wb(const Eigen::Ref<const Vec>& pose_wb_xyzwxyz);

### 5.6 Structured parameters (matrices + monotonic time)
/**
 * @brief Sets an externally generated joint trajectory (inputs in source joint order).
 * @param[in] traj_src Joint trajectory:
 *   - `t`: time [s], shape = (N,), strictly increasing.
 *   - `q`: joint positions [rad], shape = (N, n()) RowMajor (source joint order).
 *   - `qd`: joint velocities [rad/s], shape = (N, n()) RowMajor (source joint order).
 *   - `qdd`: joint accelerations [rad/s^2], shape = (N, n()) RowMajor (source joint order).
 * @param[in] delay Start delay [s].
 * @throws std::runtime_error If the trajectory is empty, shapes mismatch, or `t` is not strictly increasing.
 */
void set_joint_traj(const JointTrajectory& traj_src, double delay = 0.0);

### 5.7 Structs: document members in the type doc (keep fields comment-free)
Note: member docs like `///< ...` attach to the member symbol (good for hovering the member), not the parent type.
If you want the `struct`/`class` docs to summarize fields without adding per-field comments, list them under the type `@details`.

/**
 * @brief Joint state (model canonical order).
 *
 * @details
 * Sizes: `q`, `qd`, and `qdd` are `size = n()` (in model canonical order).
 * - `q`: Joint positions [rad].
 * - `qd`: Joint velocities [rad/s].
 * - `qdd`: Joint accelerations [rad/s^2].
 * - `t`: Sample time [s].
 */
struct JointState {
  Vec q, qd, qdd;
  double t{0.0};
};

### 5.8 Classes: include ordering + a real usage snippet (Robot)
/**
 * @brief Fast robot wrapper around dynamics + control.
 *
 * @details
 * Unless otherwise stated, joint vectors passed into setters are in *source joint order* (I/O/actuator order).
 * Internally, state is stored in *model canonical order*.
 *
 * @par Usage
 * @code{.cpp}
 * rbt_core_cpp::RobotSpec spec;
 * spec.name = "GEN3";
 * spec.urdf_path = "/path/to/robot.urdf";
 * spec.tcp_frame = "tool_frame";
 *
 * rbt_core_cpp::Robot robot = rbt_core_cpp::Robot::FromSpec(spec);
 * robot.configure_io(robot.canonical_joint_names());
 *
 * robot.set_joint_state(&q, &qd, nullptr, &t);
 * const rbt_core_cpp::Robot::Vec& tau = robot.compute_ctrl_effort();
 * @endcode
 *
 * @par Thread safety
 * Not thread-safe. External synchronization is required if accessed concurrently.
 *
 * @throws std::runtime_error If I/O mapping or inputs are invalid.
 *
 * @warning
 * Be explicit about joint vector ordering (source vs model) and sizes (`n()`, `nq()`, `nv()`).
 */
class Robot { /* ... */ };

## 6. Common expectations in this repo

1. Always document units and coordinate frames when applicable.
2. Always document expected size/shape for vector-like parameters and fields (e.g., `size = n()`, `length = num_points`), and layout/stride for matrix-like data.
3. Document nullability, ownership, and lifetime for references, pointers, views, spans. If a pointer/view is stored, say so and state what must outlive what.
4. Document ordering conventions (e.g., source joint order vs model canonical order) anywhere it affects correctness.
