import pinocchio.pinocchio_pywrap_default.linalg
import typing

__all__ = [
    "computeLargestEigenvector",
    "inv",
    "retrieveLargestEigenvalue"
]

def computeLargestEigenvector(mat_: numpy.ndarray, max_it: int = 10, rel_tol: float = 1e-08) -> numpy.ndarray:
    """
    computeLargestEigenvector( (numpy.ndarray)mat [, (int)max_it=10 [, (float)rel_tol=1e-08]]) -> numpy.ndarray :
        Compute the lagest eigenvector of a given matrix according to a given eigenvector estimate.
    """

def inv(arg1: numpy.ndarray) -> numpy.ndarray:
    """
    inv( (numpy.ndarray)arg1) -> numpy.ndarray :
        Computes the inverse of a matrix.
    """

def retrieveLargestEigenvalue(eigenvector: numpy.ndarray) -> float:
    """
    retrieveLargestEigenvalue( (numpy.ndarray)eigenvector) -> float :
        Compute the lagest eigenvalue of a given matrix. This is taking the eigenvector computed by the function computeLargestEigenvector.
    """

import numpy
_Shape = typing.Tuple[int, ...]

