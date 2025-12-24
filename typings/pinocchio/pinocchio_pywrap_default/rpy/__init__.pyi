import pinocchio.pinocchio_pywrap_default.rpy
import typing

__all__ = [
    "computeRpyJacobian",
    "computeRpyJacobianInverse",
    "computeRpyJacobianTimeDerivative",
    "matrixToRpy",
    "rotate",
    "rpyToMatrix"
]

def computeRpyJacobian(rpy_: numpy.ndarray, reference_frame: ReferenceFrame = pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> numpy.ndarray:
    """
    computeRpyJacobian( (numpy.ndarray)rpy [, (ReferenceFrame)reference_frame=pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL]) -> numpy.ndarray :
        Compute the Jacobian of the Roll-Pitch-Yaw conversion Given phi = (r, p, y) such that that R = R_z(y)R_y(p)R_x(r) and reference frame F (either LOCAL or WORLD), the Jacobian is such that omega_F = J_F(phi)phidot, where omega_F is the angular velocity expressed in frame F and J_F is the Jacobian computed with reference frame F
        Parameters:
        	rpy Roll-Pitch-Yaw vector	reference_frame  Reference frame in which the angular velocity is expressed. Notice LOCAL_WORLD_ALIGNED is equivalent to WORLD
    """

def computeRpyJacobianInverse(rpy_: numpy.ndarray, reference_frame: ReferenceFrame = pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> numpy.ndarray:
    """
    computeRpyJacobianInverse( (numpy.ndarray)rpy [, (ReferenceFrame)reference_frame=pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL]) -> numpy.ndarray :
        Compute the inverse Jacobian of the Roll-Pitch-Yaw conversion Given phi = (r, p, y) such that that R = R_z(y)R_y(p)R_x(r) and reference frame F (either LOCAL or WORLD), the Jacobian is such that omega_F = J_F(phi)phidot, where omega_F is the angular velocity expressed in frame F and J_F is the Jacobian computed with reference frame F
        Parameters:
        	rpy Roll-Pitch-Yaw vector	reference_frame  Reference frame in which the angular velocity is expressed. Notice LOCAL_WORLD_ALIGNED is equivalent to WORLD
    """

def computeRpyJacobianTimeDerivative(rpy: numpy.ndarray, rpydot_: numpy.ndarray, reference_frame: ReferenceFrame = pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> numpy.ndarray:
    """
    computeRpyJacobianTimeDerivative( (numpy.ndarray)rpy, (numpy.ndarray)rpydot [, (ReferenceFrame)reference_frame=pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL]) -> numpy.ndarray :
        Compute the time derivative of the Jacobian of the Roll-Pitch-Yaw conversion Given phi = (r, p, y) such that that R = R_z(y)R_y(p)R_x(r) and reference frame F (either LOCAL or WORLD), the Jacobian is such that omega_F = J_F(phi)phidot, where omega_F is the angular velocity expressed in frame F and J_F is the Jacobian computed with reference frame F
        Parameters:
        	rpy Roll-Pitch-Yaw vector	reference_frame  Reference frame in which the angular velocity is expressed. Notice LOCAL_WORLD_ALIGNED is equivalent to WORLD
    """

def matrixToRpy(R: numpy.ndarray) -> numpy.ndarray:
    """
    matrixToRpy( (numpy.ndarray)R) -> numpy.ndarray :
        Given a rotation matrix R, the angles (r, p, y) are given so that R = R_z(y)R_y(p)R_x(r), where R_a(theta) denotes the rotation of theta radians axis a. The angles are guaranteed to be in the ranges: r in [-pi,pi], p in[-pi/2,pi/2], y in [-pi,pi]
    """

def rotate(axis: str, angle: float) -> numpy.ndarray:
    """
    rotate( (str)axis, (float)angle) -> numpy.ndarray :
        Rotation matrix corresponding to a rotation about x, y or z e.g. R = rot('x', pi / 4): rotate pi/4 rad about x axis
    """

@typing.overload
def rpyToMatrix(roll: float, pitch: float, yaw: float) -> numpy.ndarray:
    """
    rpyToMatrix( (float)roll, (float)pitch, (float)yaw) -> numpy.ndarray :
        Given (r, p, y), the rotation is given as R = R_z(y)R_y(p)R_x(r), where R_a(theta) denotes the rotation of theta radians axis a
    """
@typing.overload
def rpyToMatrix(rpy: numpy.ndarray) -> numpy.ndarray:
    pass

import numpy
import pinocchio.pinocchio_pywrap_default.ReferenceFrame
_Shape = typing.Tuple[int, ...]

