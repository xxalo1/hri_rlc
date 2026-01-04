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
2. If it is not obvious, add `@details` with key behavior or constraints.

### Add only when relevant
`@param` for parameters (include units/frames/ranges when meaningful).  
Use direction tags consistently:
  @param[in] x ...
  @param[out] y ...
  @param[in,out] z ...

`@return` only if the function returns a value.

`@throws` only if it throws.

`@pre` only for real preconditions (caller must satisfy).  
`@post` only for real guarantees after success.

`@warning` only for misuse that can silently break behavior or safety.  
`@note` for clarifications that are not contracts.

`@see` / `@sa` to link related types or functions.

## 3. Usage examples and code blocks

Doxygen does not have a standard `@usage` tag. Use a section instead:

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

1. If the name and signature already explain it, keep docs to `@brief` only.
2. Do not restate types (avoid “param is a double”).
3. Keep `@brief` one line. Put longer text in `@details`.

## 5. Minimal templates

### 5.1 Simple function (no params, no return)
/// @brief Resets the internal state to defaults.
void Reset();

### 5.2 Function with params and return
/**
 * @brief Computes joint torques for the current state.
 * @param[in] q Joint positions [rad].
 * @param[in] dq Joint velocities [rad/s].
 * @return Torque command [N·m].
 */
Vec ComputeTorque(const Vec& q, const Vec& dq);

### 5.3 Function with a real precondition and possible throw
/**
 * @brief Loads configuration from disk.
 * @param[in] path Config file path.
 * @pre path is non-empty.
 * @throws std::runtime_error If the file cannot be read or parsed.
 */
void LoadConfig(std::string_view path);

/**
 * @brief Low-allocation controller for joint-space commands.
 *
 * @details
 * Owns the control state and produces torque commands from the latest robot state.
 * Document any non-obvious ownership, lifetime, and invariants here.
 *
 * @par Usage
 * @code{.cpp}
 * Controller ctrl(params);
 * ctrl.Reset();
 *
 * ctrl.UpdateState(q, dq);
 * const auto tau = ctrl.Step(dt_sec);
 * @endcode
 *
 * @par Thread safety
 * Not thread-safe. External synchronization is required if accessed concurrently.
 *
 * @warning
 * Callers must provide consistent units and ordering for joint vectors.
 */
class Controller { /* ... */ };

## 6. Common expectations in this repo

1. Always document units and coordinate frames when applicable.
2. Document ownership and lifetime for references, pointers, views, spans.