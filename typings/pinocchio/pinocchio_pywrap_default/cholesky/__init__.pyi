import pinocchio.pinocchio_pywrap_default.cholesky
import typing

__all__ = [
    "computeMinv",
    "decompose",
    "solve"
]

def computeMinv(Model: Model, Data: Data) -> numpy.ndarray:
    """
    computeMinv( (Model)Model, (Data)Data) -> numpy.ndarray :
        Returns the inverse of the joint space inertia matrix using the results of the Cholesky decomposition
        performed by cholesky.decompose. The result is stored in data.Minv.
    """

def decompose(Model: Model, Data: Data) -> numpy.ndarray:
    """
    decompose( (Model)Model, (Data)Data) -> numpy.ndarray :
        Computes the Cholesky decomposition of the joint space inertia matrix M contained in data.
        The upper triangular part of data.M should have been filled first by calling crba, or any related algorithms.
    """

def solve(Model: Model, Data: Data, v: numpy.ndarray) -> numpy.ndarray:
    """
    solve( (Model)Model, (Data)Data, (numpy.ndarray)v) -> numpy.ndarray :
        Returns the solution $x$ of $ M x = y $ using the Cholesky decomposition stored in data given the entry $ y $.
    """

import numpy
_Shape = typing.Tuple[int, ...]

