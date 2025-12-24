#!/usr/bin/env python3
"""
Pinocchio direct-type inspector (NO np.asarray / NO numpy conversion).

Goal
- Print the *direct* Python types returned by pin.Data fields.
- For numpy arrays: show shape + dtype (they already exist as ndarray).
- For Pinocchio StdVec_* containers: show length and a few element types.
- For SE3 / Motion elements: print the types of their key members (rotation/translation/vector)
  without converting to numpy.

Run
  python3 pin_direct_inspect.py
"""

from __future__ import annotations

import sys
from typing import Any, Optional, Union

import numpy as np

try:
    import pinocchio as pin
except Exception as e:
    print("Failed to import pinocchio:", repr(e))
    sys.exit(1)

FrameKey = Union[str, int]


def inspect_direct(name: str, obj: Any, n: int = 3) -> None:
    print(f"\n{name}")
    print("  type:", type(obj))

    # Already numpy arrays: show shape/dtype directly (no conversion)
    if hasattr(obj, "shape") and hasattr(obj, "dtype"):
        try:
            print("  shape:", obj.shape)
            print("  dtype:", obj.dtype)
        except Exception as e:
            print("  (shape/dtype unavailable):", repr(e))
        return

    # Try to treat as a sized container (StdVec_*)
    try:
        L = len(obj)
    except TypeError:
        print("  not a sized container")
        return
    except Exception as e:
        print("  len() failed:", repr(e))
        return

    print("  len:", L)

    for i in range(min(n, L)):
        try:
            e = obj[i]
        except Exception as ex:
            print(f"  [{i}] <indexing failed>:", repr(ex))
            continue

        print(f"  [{i}] type:", type(e))

        # SE3 element: check rotation / translation attributes
        if hasattr(e, "rotation") and hasattr(e, "translation"):
            try:
                R = e.rotation
                t = e.translation
                print("     looks_like: SE3")
                print("     rotation type:", type(R))
                if hasattr(R, "shape"):
                    print("     rotation shape:", R.shape)
                if hasattr(R, "dtype"):
                    print("     rotation dtype:", R.dtype)

                print("     translation type:", type(t))
                if hasattr(t, "shape"):
                    print("     translation shape:", t.shape)
                if hasattr(t, "dtype"):
                    print("     translation dtype:", t.dtype)

                # Homogeneous matrix accessor (still no conversion)
                if hasattr(e, "homogeneous"):
                    H = e.homogeneous
                    print("     homogeneous type:", type(H))
                    if hasattr(H, "shape"):
                        print("     homogeneous shape:", H.shape)
                    if hasattr(H, "dtype"):
                        print("     homogeneous dtype:", H.dtype)
                elif hasattr(e, "toHomogeneousMatrix"):
                    H = e.toHomogeneousMatrix()
                    print("     toHomogeneousMatrix() type:", type(H))
                    if hasattr(H, "shape"):
                        print("     toHomogeneousMatrix() shape:", H.shape)
                    if hasattr(H, "dtype"):
                        print("     toHomogeneousMatrix() dtype:", H.dtype)
            except Exception as ex:
                print("     (SE3 detail failed):", repr(ex))

        # Motion / Force element: check .vector
        if hasattr(e, "vector"):
            try:
                v = e.vector
                print("     looks_like: Motion/Force (has .vector)")
                print("     vector type:", type(v))
                if hasattr(v, "shape"):
                    print("     vector shape:", v.shape)
                if hasattr(v, "dtype"):
                    print("     vector dtype:", v.dtype)
            except Exception as ex:
                print("     (vector detail failed):", repr(ex))


def build_model() -> tuple[pin.Model, pin.Data, Optional[int]]:
    """
    Build a small model:
    - prefer pin.buildSampleModelManipulator() if available
    - fallback to a 3R chain if not
    Returns: (model, data, ee_frame_id_or_None)
    """
    if hasattr(pin, "buildSampleModelManipulator"):
        model = pin.buildSampleModelManipulator()
        data = model.createData()
        ee_frame_id = model.nframes - 1 if model.nframes > 0 else None
        return model, data, ee_frame_id

    # Fallback: simple 3R chain
    model = pin.Model()
    I3 = np.eye(3)
    Z3 = np.zeros(3)

    def inertia_box(m: float) -> pin.Inertia:
        J = np.diag([0.01 * m, 0.01 * m, 0.01 * m])
        return pin.Inertia(m, Z3, J)

    j1 = model.addJoint(0, pin.JointModelRZ(), pin.SE3(I3, np.array([0.0, 0.0, 0.0])), "joint1")
    model.appendBodyToJoint(j1, inertia_box(1.0), pin.SE3.Identity())

    j2 = model.addJoint(j1, pin.JointModelRY(), pin.SE3(I3, np.array([0.3, 0.0, 0.0])), "joint2")
    model.appendBodyToJoint(j2, inertia_box(0.8), pin.SE3.Identity())

    j3 = model.addJoint(j2, pin.JointModelRY(), pin.SE3(I3, np.array([0.3, 0.0, 0.0])), "joint3")
    model.appendBodyToJoint(j3, inertia_box(0.6), pin.SE3.Identity())

    data = model.createData()
    ee_frame_id = None
    return model, data, ee_frame_id


def main() -> None:
    model, data, ee_frame_id = build_model()

    print("\nModel sizes")
    print("  nq:", model.nq)
    print("  nv:", model.nv)
    print("  njoints:", model.njoints)
    print("  nframes:", model.nframes)

    # State
    q = pin.neutral(model)
    qd = np.random.randn(model.nv)
    qdd = np.random.randn(model.nv)

    # Kinematics: populate data.oMi, data.oMf, data.v, data.a
    pin.forwardKinematics(model, data, q, qd, qdd)
    pin.updateFramePlacements(model, data)

    # Jacobians/time-variation (optional)
    ref = None
    if hasattr(pin, "ReferenceFrame"):
        ref = getattr(pin.ReferenceFrame, "LOCAL_WORLD_ALIGNED", None) or getattr(pin.ReferenceFrame, "WORLD", None)

    if hasattr(pin, "computeJointJacobians"):
        try:
            pin.computeJointJacobians(model, data, q)
        except Exception:
            pass

    if hasattr(pin, "computeJointJacobiansTimeVariation"):
        try:
            pin.computeJointJacobiansTimeVariation(model, data, q, qd)
        except Exception:
            pass

    # Dynamics: populate common terms
    if hasattr(pin, "computeAllTerms"):
        try:
            pin.computeAllTerms(model, data, q, qd)
        except Exception:
            # fallback: populate a few explicitly
            if hasattr(pin, "crba"):
                try:
                    pin.crba(model, data, q)
                except Exception:
                    pass
            if hasattr(pin, "nonLinearEffects"):
                try:
                    pin.nonLinearEffects(model, data, q, qd)
                except Exception:
                    pass
            if hasattr(pin, "computeGeneralizedGravity"):
                try:
                    pin.computeGeneralizedGravity(model, data, q)
                except Exception:
                    pass

    # ---- Direct inspection (NO conversion) ----
    inspect_direct("data.oMi", data.oMi)
    inspect_direct("data.oMf", data.oMf)
    if hasattr(data, "v"):
        inspect_direct("data.v", data.v)
    if hasattr(data, "a"):
        inspect_direct("data.a", data.a)
    if hasattr(data, "com"):
        inspect_direct("data.com", data.com)

    # Numpy arrays directly stored in data
    for field in ("M", "Minv", "g", "nle", "Ag", "dAg", "J", "dJ", "ddq"):
        if hasattr(data, field):
            inspect_direct(f"data.{field}", getattr(data, field))

    # Pinocchio objects (Force, Inertia) that may have .vector
    for field in ("hg", "Ig"):
        if hasattr(data, field):
            inspect_direct(f"data.{field}", getattr(data, field))

    # Also demonstrate direct jacobian returns (these are ndarray)
    if ref is not None and hasattr(pin, "getJointJacobian"):
        try:
            Jj = pin.getJointJacobian(model, data, model.njoints - 1, ref)
            inspect_direct("pin.getJointJacobian(...)", Jj)
        except Exception as e:
            print("\ngetJointJacobian failed:", repr(e))

    if ee_frame_id is not None and ref is not None and hasattr(pin, "getFrameJacobian"):
        try:
            Jf = pin.getFrameJacobian(model, data, ee_frame_id, ref)
            inspect_direct("pin.getFrameJacobian(...)", Jf)
        except Exception as e:
            print("\ngetFrameJacobian failed:", repr(e))


if __name__ == "__main__":
    main()
