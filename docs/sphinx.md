# Python Docstring Rules (Sphinx / NumPy style)

This repo uses **NumPy-style docstrings** for public Python APIs. This style is readable in editors and renders well with Sphinx via `sphinx.ext.napoleon`.

## 1. Basic format

- Use triple double quotes: `"""..."""`.
- First line is a 1‑sentence summary (what it does, observable behavior).
- Add a blank line after the summary.
- Keep it brief; add more only when the API is non-obvious or easy to misuse.

## 2. Sections (use when applicable)

- `Parameters` for any function/method with parameters.
- `Returns` for any non-`None` return value.
- `Raises` for any exception the function can raise (including from validations you call).
- `Notes` only for important constraints: units, frames, ordering, statefulness, performance.
- `Examples` only for nontrivial usage.

## 3. Parameters: types + size/shape + units

Write parameter entries like:
- `name : type`
- `name : type, optional`
- Arrays: include shape/length and ordering in the type line when possible.

Type naming (what top libraries do):
- Use `ndarray` in docstrings for NumPy arrays (NumPy/SciPy convention). Use `numpy.ndarray` only when disambiguation matters.
- Use `Tensor` in docstrings for PyTorch tensors (PyTorch convention). Use `torch.Tensor` only when disambiguation matters.

Conventions used in this repo:
- Vectors: `ndarray, shape (n,)`
- Matrices: `ndarray, shape (N, n)`
- Sequences: `Sequence[str], length n`
- Pose vectors: `ndarray, shape (7,)` with explicit ordering `[x, y, z, qw, qx, qy, qz]`
- Always include units/frames/order when it matters (e.g., `[rad]`, `[rad/s]`, `[s]`, `world_from_base`, source joint order).

Optional parameters (`None` allowed):
- Mark as `optional` and define behavior when omitted (e.g., “If None, no update” / “If None, field is omitted”).

## 4. Returns: meaning + size/shape

- Document return type and any size/shape/units.
- If returning a view/cached internal array, document lifetime/invalidation rules.

## 5. Preconditions and errors

- If the function checks and raises (e.g., shape checks), document it under `Raises`.
- If something must be true but is not checked, document it as a real constraint (prefer checking when feasible).

## 6. Type hints vs docstrings

- Prefer real Python type hints in the signature.
- Use docstrings to add the missing information type hints do not capture well: units, frames, ordering, shapes, constraints, side effects.

## 7. Minimal templates (examples from this repo)

### 7.1 Simple method (no params, no return)
```python
def clear_traj(self) -> None:
    """Clear the currently set trajectory."""
```

### 7.2 Array parameter with shape + error (`Robot.set_base_pose`)
```python
def set_base_pose(self, pose_wb: FloatArray) -> None:
    """
    Set base pose in world frame and update gravity in the base/model frame.

    Parameters
    ----------
    pose_wb : ndarray, shape (7,)
        World-from-base pose `[x, y, z, qw, qx, qy, qz]`. Position [m]. Quaternion
        is `[w, x, y, z]` and is normalized internally.

    Raises
    ------
    ValueError
        If `pose_wb.shape != (7,)` or the quaternion has zero norm.
    """
```

### 7.3 Optional arrays + shape constraints (`ros_utils.msg_conv.to_joint_traj_msg`)
```python
def to_joint_traj_msg(
    stamp: float,
    joint_names: Sequence[str],
    time_from_start: FloatArray,
    positions: FloatArray,
    velocities: FloatArray | None = None,
    accelerations: FloatArray | None = None,
) -> JointTrajectory:
    """
    Build a `JointTrajectory` message from joint positions (and optional derivatives).

    Parameters
    ----------
    stamp : float
        Start time [s].
    joint_names : Sequence[str]
        Joint names, length n (matches the column ordering of the inputs).
    time_from_start : ndarray, shape (N,)
        Time from `stamp` for each sample [s].
    positions : ndarray, shape (N, n) or (n,)
        Joint positions [rad].
    velocities : ndarray, shape (N, n) or (n,), optional
        Joint velocities [rad/s]. If None, omit velocities in the message.
    accelerations : ndarray, shape (N, n) or (n,), optional
        Joint accelerations [rad/s^2]. If None, omit accelerations in the message.

    Returns
    -------
    JointTrajectory
        Populated ROS message.

    Raises
    ------
    ValueError
        If array shapes are incompatible with `(N, n)`.
    """
```

### 7.4 Dataclasses: document public fields (`RobotSpec`)
```python
@dataclass
class RobotSpec:
    """
    Minimal robot specification.

    Attributes
    ----------
    name : str
        Robot name.
    urdf : pathlib.Path
        Path to the URDF file.
    tcp_frame : str
        Tool frame name in the URDF.
    """
```
