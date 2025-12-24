import pinocchio.pinocchio_pywrap_default.serialization
import typing

__all__ = [
    "StaticBuffer",
    "StreamBuffer",
    "buffer_copy",
    "loadFromBinary",
    "saveToBinary"
]

class StaticBuffer(Boost.Python.instance):
    """
    Static buffer to save/load serialized objects in binary mode with pre-allocated memory.
    """
    def __init__(self, size: int) -> None: 
        """
        __init__( (object)self, (int)size) -> None :
            Default constructor from a given size capacity.
        """
    @staticmethod
    def reserve(arg1: StaticBuffer, new_size: int) -> None: 
        """
        reserve( (StaticBuffer)arg1, (int)new_size) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_size.
        """
    def size(self) -> int: 
        """
        size( (StaticBuffer)self) -> int :
            Get the size of the input sequence.
        """
    __instance_size__ = 56
    pass

class StreamBuffer(Boost.Python.instance):
    """
    Stream buffer to save/load serialized objects in binary mode.
    """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None :
            Default constructor.
        """
    @staticmethod
    def max_size(arg1: StreamBuffer) -> int: 
        """
        max_size( (StreamBuffer)arg1) -> int :
            Get the maximum size of the StreamBuffer.
        """
    @staticmethod
    def prepare(arg1: StreamBuffer, arg2: int) -> StreamBuffer: 
        """
        prepare( (StreamBuffer)arg1, (int)arg2) -> StreamBuffer :
            Reserve data.
        """
    @staticmethod
    def size(arg1: StreamBuffer) -> int: 
        """
        size( (StreamBuffer)arg1) -> int :
            Get the size of the input sequence.
        """
    @staticmethod
    def tobytes(arg1: StreamBuffer) -> object: 
        """
        tobytes( (StreamBuffer)arg1) -> object :
            Returns the content of *this as a byte sequence.
        """
    @staticmethod
    def view(arg1: StreamBuffer) -> object: 
        """
        view( (StreamBuffer)arg1) -> object :
            Returns the content of *this as a memory view.
        """
    __instance_size__ = 120
    pass

def buffer_copy(dest: StreamBuffer, source: StreamBuffer) -> None:
    """
    buffer_copy( (StreamBuffer)dest, (StreamBuffer)source) -> None :
        Copy bytes from a source buffer to a target buffer.
    """

@typing.overload
def loadFromBinary(object: Data, static_buffer: StaticBuffer) -> None:
    """
    loadFromBinary( (StdVec_SE3)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    """
@typing.overload
def loadFromBinary(object: Data, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: GeometryData, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: GeometryData, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: GeometryModel, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: GeometryModel, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: GeometryObject, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: GeometryObject, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: Model, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: Model, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Bool, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Bool, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_CollisionPair, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_CollisionPair, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Force, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Force, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Frame, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Frame, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Index, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Index, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_IndexVector, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_IndexVector, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Inertia, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Inertia, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Matrix6x, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Matrix6x, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Motion, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Motion, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_SE3, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_SE3, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Scalar, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Scalar, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_StdString, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_StdString, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Symmetric3, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Symmetric3, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Vector3, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_Vector3, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_int, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def loadFromBinary(object: StdVec_int, stream_buffer: StreamBuffer) -> None:
    pass

@typing.overload
def saveToBinary(object: Data, static_buffer: StaticBuffer) -> None:
    """
    saveToBinary( (StdVec_SE3)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    """
@typing.overload
def saveToBinary(object: Data, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: GeometryData, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: GeometryData, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: GeometryModel, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: GeometryModel, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: GeometryObject, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: GeometryObject, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: Model, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: Model, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Bool, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Bool, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_CollisionPair, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_CollisionPair, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Force, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Force, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Frame, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Frame, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Index, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Index, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_IndexVector, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_IndexVector, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Inertia, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Inertia, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Matrix6x, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Matrix6x, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Motion, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Motion, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_SE3, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_SE3, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Scalar, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Scalar, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_StdString, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_StdString, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Symmetric3, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Symmetric3, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Vector3, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_Vector3, stream_buffer: StreamBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_int, static_buffer: StaticBuffer) -> None:
    pass
@typing.overload
def saveToBinary(object: StdVec_int, stream_buffer: StreamBuffer) -> None:
    pass

import Boost.Python

