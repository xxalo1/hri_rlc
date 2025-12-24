import pinocchio
import coal
import typing
from typing import Mapping, Sequence
from numpy.typing import NDArray
FloatVec3 = NDArray[numpy.floating]   # shape (3,)
FloatVec6 = NDArray[numpy.floating]   # shape (6,)
FloatMat6 = NDArray[numpy.floating]   # shape (6, 6)
FloatMat4 = NDArray[numpy.floating]   # shape (4, 4)

__all__ = [
    "ACCELERATION",
    "ADMMContactSolver",
    "ARG0",
    "ARG1",
    "ARG2",
    "ARG3",
    "ARG4",
    "AngleAxis",
    "ArgumentPosition",
    "BODY",
    "BaumgarteCorrectorParameters",
    "COLLISION",
    "CollisionPair",
    "ContactCholeskyDecomposition",
    "ContactType",
    "Convention",
    "CoulombFrictionCone",
    "Data",
    "DelassusCholeskyExpression",
    "DelassusOperatorDense",
    "DelassusOperatorSparse",
    "DualCoulombFrictionCone",
    "Exception",
    "FIXED_JOINT",
    "Force",
    "Frame",
    "FrameType",
    "GeometryData",
    "GeometryModel",
    "GeometryNoMaterial",
    "GeometryObject",
    "GeometryPhongMaterial",
    "GeometryType",
    "Hlog3",
    "Inertia",
    "JOINT",
    "Jexp3",
    "Jexp6",
    "Jlog3",
    "Jlog6",
    "JointData",
    "JointDataComposite",
    "JointDataFreeFlyer",
    "JointDataHX",
    "JointDataHY",
    "JointDataHZ",
    "JointDataHelicalUnaligned",
    "JointDataMimic_JointDataRX",
    "JointDataMimic_JointDataRY",
    "JointDataMimic_JointDataRZ",
    "JointDataPX",
    "JointDataPY",
    "JointDataPZ",
    "JointDataPlanar",
    "JointDataPrismaticUnaligned",
    "JointDataRUBX",
    "JointDataRUBY",
    "JointDataRUBZ",
    "JointDataRX",
    "JointDataRY",
    "JointDataRZ",
    "JointDataRevoluteUnaligned",
    "JointDataRevoluteUnboundedUnalignedTpl",
    "JointDataSpherical",
    "JointDataSphericalZYX",
    "JointDataTranslation",
    "JointDataUniversal",
    "JointModel",
    "JointModelComposite",
    "JointModelFreeFlyer",
    "JointModelHX",
    "JointModelHY",
    "JointModelHZ",
    "JointModelHelicalUnaligned",
    "JointModelMimic_JointModelRX",
    "JointModelMimic_JointModelRY",
    "JointModelMimic_JointModelRZ",
    "JointModelPX",
    "JointModelPY",
    "JointModelPZ",
    "JointModelPlanar",
    "JointModelPrismaticUnaligned",
    "JointModelRUBX",
    "JointModelRUBY",
    "JointModelRUBZ",
    "JointModelRX",
    "JointModelRY",
    "JointModelRZ",
    "JointModelRevoluteUnaligned",
    "JointModelRevoluteUnboundedUnaligned",
    "JointModelSpherical",
    "JointModelSphericalZYX",
    "JointModelTranslation",
    "JointModelUniversal",
    "KinematicLevel",
    "LOCAL",
    "LOCAL_WORLD_ALIGNED",
    "LanczosDecomposition",
    "LieGroup",
    "LogCholeskyParameters",
    "LogLevel",
    "Model",
    "Motion",
    "OP_FRAME",
    "PGSContactSolver",
    "PINOCCHIO_MAJOR_VERSION",
    "PINOCCHIO_MINOR_VERSION",
    "PINOCCHIO_PATCH_VERSION",
    "POSITION",
    "PowerIterationAlgo",
    "ProximalSettings",
    "PseudoInertia",
    "Quaternion",
    "ReferenceFrame",
    "RigidConstraintData",
    "RigidConstraintModel",
    "RobotWrapper",
    "SE3",
    "SE3ToXYZQUAT",
    "SE3ToXYZQUATtuple",
    "SENSOR",
    "ScalarType",
    "SolverStats",
    "StdMap_String_VectorXd",
    "StdVec_Bool",
    "StdVec_CollisionPair",
    "StdVec_CoulombFrictionCone",
    "StdVec_Double",
    "StdVec_DualCoulombFrictionCone",
    "StdVec_Force",
    "StdVec_Frame",
    "StdVec_GeometryModel",
    "StdVec_GeometryObject",
    "StdVec_Index",
    "StdVec_IndexVector",
    "StdVec_Inertia",
    "StdVec_JointDataVector",
    "StdVec_JointModelVector",
    "StdVec_Matrix6",
    "StdVec_Matrix6x",
    "StdVec_MatrixXs",
    "StdVec_Motion",
    "StdVec_RigidConstraintData",
    "StdVec_RigidConstraintModel",
    "StdVec_SE3",
    "StdVec_Scalar",
    "StdVec_StdString",
    "StdVec_Symmetric3",
    "StdVec_Vector3",
    "StdVec_VectorXb",
    "StdVec_int",
    "Symmetric3",
    "TridiagonalSymmetricMatrix",
    "VELOCITY",
    "VISUAL",
    "WITH_CPPAD",
    "WITH_HPP_FCL",
    "WITH_HPP_FCL_BINDINGS",
    "WITH_OPENMP",
    "WITH_SDFORMAT",
    "WITH_URDFDOM",
    "WORLD",
    "XAxis",
    "XYZQUATToSE3",
    "YAxis",
    "ZAxis",
    "aba",
    "annotations",
    "appendModel",
    "bodyRegressor",
    "boost_type_index",
    "buildGeomFromMJCF",
    "buildGeomFromUrdf",
    "buildGeomFromUrdfString",
    "buildModelFromMJCF",
    "buildModelFromUrdf",
    "buildModelFromXML",
    "buildModelsFromMJCF",
    "buildModelsFromSdf",
    "buildModelsFromUrdf",
    "buildReducedModel",
    "buildSampleModelHumanoid",
    "buildSampleModelHumanoidRandom",
    "buildSampleModelManipulator",
    "ccrba",
    "centerOfMass",
    "checkVersionAtLeast",
    "cholesky",
    "classicAcceleration",
    "computeABADerivatives",
    "computeAllTerms",
    "computeCentroidalDynamicsDerivatives",
    "computeCentroidalMap",
    "computeCentroidalMapTimeVariation",
    "computeCentroidalMomentum",
    "computeCentroidalMomentumTimeVariation",
    "computeComplementarityShift",
    "computeConeProjection",
    "computeConstraintDynamicsDerivatives",
    "computeContactForces",
    "computeCoriolisMatrix",
    "computeDampedDelassusMatrixInverse",
    "computeDelassusMatrix",
    "computeDualConeProjection",
    "computeForwardKinematicsDerivatives",
    "computeFrameJacobian",
    "computeFrameKinematicRegressor",
    "computeGeneralizedGravity",
    "computeGeneralizedGravityDerivatives",
    "computeImpulseDynamicsDerivatives",
    "computeJointJacobian",
    "computeJointJacobians",
    "computeJointJacobiansTimeVariation",
    "computeJointKinematicRegressor",
    "computeJointTorqueRegressor",
    "computeKKTContactDynamicMatrixInverse",
    "computeKineticEnergy",
    "computeKineticEnergyRegressor",
    "computeMechanicalEnergy",
    "computeMinverse",
    "computePotentialEnergy",
    "computePotentialEnergyRegressor",
    "computePrimalFeasibility",
    "computeRNEADerivatives",
    "computeReprojectionError",
    "computeStaticRegressor",
    "computeStaticTorque",
    "computeStaticTorqueDerivatives",
    "computeSubtreeMasses",
    "computeSupportedForceByFrame",
    "computeSupportedInertiaByFrame",
    "computeTotalMass",
    "constraintDynamics",
    "contactInverseDynamics",
    "crba",
    "createDatas",
    "dDifference",
    "dIntegrate",
    "dIntegrateTransport",
    "dccrba",
    "deprecated",
    "difference",
    "distance",
    "exp",
    "exp3",
    "exp3_quat",
    "exp6",
    "exp6_quat",
    "explog",
    "findCommonAncestor",
    "forwardDynamics",
    "forwardKinematics",
    "frameBodyRegressor",
    "frameJacobianTimeVariation",
    "framesForwardKinematics",
    "getAcceleration",
    "getCenterOfMassVelocityDerivatives",
    "getCentroidalDynamicsDerivatives",
    "getClassicalAcceleration",
    "getConstraintJacobian",
    "getConstraintsJacobian",
    "getCoriolisMatrix",
    "getFrameAcceleration",
    "getFrameAccelerationDerivatives",
    "getFrameClassicalAcceleration",
    "getFrameJacobian",
    "getFrameJacobianTimeVariation",
    "getFrameVelocity",
    "getFrameVelocityDerivatives",
    "getJacobianSubtreeCenterOfMass",
    "getJointAccelerationDerivatives",
    "getJointJacobian",
    "getJointJacobianTimeVariation",
    "getJointVelocityDerivatives",
    "getKKTContactDynamicMatrixInverse",
    "getPointClassicAccelerationDerivatives",
    "getPointVelocityDerivatives",
    "getVelocity",
    "impulseDynamics",
    "initConstraintDynamics",
    "inspect",
    "integrate",
    "interpolate",
    "isNormalized",
    "isSameConfiguration",
    "jacobianCenterOfMass",
    "jacobianSubtreeCenterOfMass",
    "jointBodyRegressor",
    "liegroups",
    "linalg",
    "loadReferenceConfigurations",
    "loadReferenceConfigurationsFromXML",
    "loadRotorParameters",
    "log",
    "log3",
    "log6",
    "log6_quat",
    "map_indexing_suite_StdMap_String_VectorXd_entry",
    "module_info",
    "neutral",
    "nle",
    "nonLinearEffects",
    "normalize",
    "numpy",
    "pin",
    "pinocchio_pywrap_default",
    "printVersion",
    "randomConfiguration",
    "removeCollisionPairs",
    "removeCollisionPairsFromXML",
    "rnea",
    "robot_wrapper",
    "rpy",
    "seed",
    "serialization",
    "sharedMemory",
    "shortcuts",
    "skew",
    "skewSquare",
    "squaredDistance",
    "std_type_index",
    "submodules",
    "sys",
    "unSkew",
    "updateFramePlacement",
    "updateFramePlacements",
    "updateGeometryPlacements",
    "updateGlobalPlacements",
    "utils"
]

class ADMMContactSolver(Boost.Python.instance):
    """
    Alternating Direction Method of Multi-pliers solver for contact dynamics.
    """
    def __init__(self, problem_dim_: int, mu_prox: float = 1e-06, tau: float = 0.5, rho_power: float = 0.2, rho_power_factor: float = 0.05, ratio_primal_dual: float = 10.0, max_it_largest_eigen_value_solver: int = 20) -> None: 
        """
        __init__( (object)self, (int)problem_dim [, (float)mu_prox=1e-06 [, (float)tau=0.5 [, (float)rho_power=0.2 [, (float)rho_power_factor=0.05 [, (float)ratio_primal_dual=10.0 [, (int)max_it_largest_eigen_value_solver=20]]]]]]) -> None :
            Default constructor.
        """
    @staticmethod
    def computeRho(L: float, m: float, rho_power: float) -> float: 
        """
        computeRho( (float)L, (float)m, (float)rho_power) -> float :
            Compute the penalty ADMM value from the current largest and lowest eigenvalues and the scaling spectral factor.
        """
    @staticmethod
    def computeRhoPower(L: float, m: float, rho: float) -> float: 
        """
        computeRhoPower( (float)L, (float)m, (float)rho) -> float :
            Compute the  scaling spectral factor of the ADMM penalty term from the current largest and lowest eigenvalues and the ADMM penalty term.
        """
    def getAbsoluteConvergenceResidual(self) -> float: 
        """
        getAbsoluteConvergenceResidual( (ADMMContactSolver)self) -> float :
            Returns the value of the absolute residual value corresponding to the contact complementary conditions.
        """
    def getAbsolutePrecision(self) -> float: 
        """
        getAbsolutePrecision( (ADMMContactSolver)self) -> float :
            Get the absolute precision requested.
        """
    def getCholeskyUpdateCount(self) -> int: 
        """
        getCholeskyUpdateCount( (ADMMContactSolver)self) -> int :
            Returns the number of updates of the Cholesky factorization due to rho updates.
        """
    def getDualSolution(self) -> object: 
        """
        getDualSolution( (ADMMContactSolver)self) -> object :
            Returns the dual solution of the problem.
        """
    def getIterationCount(self) -> int: 
        """
        getIterationCount( (ADMMContactSolver)self) -> int :
            Get the number of iterations achieved by the PGS algorithm.
        """
    def getMaxIterations(self) -> int: 
        """
        getMaxIterations( (ADMMContactSolver)self) -> int :
            Get the maximum number of iterations allowed.
        """
    def getPowerIterationAlgo(self) -> PowerIterationAlgo: 
        """
        getPowerIterationAlgo( (ADMMContactSolver)self) -> PowerIterationAlgo
        """
    def getPrimalSolution(self) -> object: 
        """
        getPrimalSolution( (ADMMContactSolver)self) -> object :
            Returns the primal solution of the problem.
        """
    def getProximalValue(self) -> float: 
        """
        getProximalValue( (ADMMContactSolver)self) -> float :
            Get the proximal value.
        """
    def getRatioPrimalDual(self) -> float: 
        """
        getRatioPrimalDual( (ADMMContactSolver)self) -> float :
            Get the primal/dual ratio.
        """
    def getRelativeConvergenceResidual(self) -> float: 
        """
        getRelativeConvergenceResidual( (ADMMContactSolver)self) -> float :
            Returns the value of the relative residual value corresponding to the difference between two successive iterates (infinity norms).
        """
    def getRelativePrecision(self) -> float: 
        """
        getRelativePrecision( (ADMMContactSolver)self) -> float :
            Get the relative precision requested.
        """
    def getRho(self) -> float: 
        """
        getRho( (ADMMContactSolver)self) -> float :
            Get the ADMM penalty value.
        """
    def getRhoPower(self) -> float: 
        """
        getRhoPower( (ADMMContactSolver)self) -> float :
            Get the power associated to the problem conditionning.
        """
    def getRhoPowerFactor(self) -> float: 
        """
        getRhoPowerFactor( (ADMMContactSolver)self) -> float :
            Get the power factor associated to the problem conditionning.
        """
    def getStats(self) -> SolverStats: 
        """
        getStats( (ADMMContactSolver)self) -> SolverStats
        """
    def getTau(self) -> float: 
        """
        getTau( (ADMMContactSolver)self) -> float :
            Get the tau linear scaling factor.
        """
    def setAbsolutePrecision(self, absolute_precision: float) -> None: 
        """
        setAbsolutePrecision( (ADMMContactSolver)self, (float)absolute_precision) -> None :
            Set the absolute precision for the problem.
        """
    def setMaxIterations(self, max_it: int) -> None: 
        """
        setMaxIterations( (ADMMContactSolver)self, (int)max_it) -> None :
            Set the maximum number of iterations.
        """
    def setProximalValue(self, mu: float) -> None: 
        """
        setProximalValue( (ADMMContactSolver)self, (float)mu) -> None :
            Set the proximal value.
        """
    def setRatioPrimalDual(self, ratio_primal_dual: float) -> None: 
        """
        setRatioPrimalDual( (ADMMContactSolver)self, (float)ratio_primal_dual) -> None :
            Set the primal/dual ratio.
        """
    def setRelativePrecision(self, relative_precision: float) -> None: 
        """
        setRelativePrecision( (ADMMContactSolver)self, (float)relative_precision) -> None :
            Set the relative precision for the problem.
        """
    def setRho(self, rho: float) -> None: 
        """
        setRho( (ADMMContactSolver)self, (float)rho) -> None :
            Set the ADMM penalty value.
        """
    def setRhoPower(self, rho_power: float) -> None: 
        """
        setRhoPower( (ADMMContactSolver)self, (float)rho_power) -> None :
            Set the power associated to the problem conditionning.
        """
    def setRhoPowerFactor(self, rho_power_factor: float) -> None: 
        """
        setRhoPowerFactor( (ADMMContactSolver)self, (float)rho_power_factor) -> None :
            Set the power factor associated to the problem conditionning.
        """
    def setTau(self, tau: float) -> None: 
        """
        setTau( (ADMMContactSolver)self, (float)tau) -> None :
            Set the tau linear scaling factor.
        """
    @typing.overload
    def solve(self, delassus: DelassusCholeskyExpression, g: numpy.ndarray, cones: StdVec_CoulombFrictionCone, R_: numpy.ndarray, primal_solution: numpy.ndarray = None, dual_solution: numpy.ndarray = None, compute_largest_eigen_values: bool = True, stat_record: bool = False) -> bool: 
        """
        solve( (ADMMContactSolver)self, (DelassusCholeskyExpression)delassus, (numpy.ndarray)g, (StdVec_CoulombFrictionCone)cones, (numpy.ndarray)R [, (numpy.ndarray)primal_solution=None [, (numpy.ndarray)dual_solution=None [, (bool)compute_largest_eigen_values=True [, (bool)stat_record=False]]]]) -> bool :
            Solve the constrained conic problem, starting from the optional initial guess.
        """
    @typing.overload
    def solve(self, delassus: DelassusOperatorDense, g: numpy.ndarray, cones: StdVec_CoulombFrictionCone, R_: numpy.ndarray, primal_solution: numpy.ndarray = None, dual_solution: numpy.ndarray = None, compute_largest_eigen_values: bool = True, stat_record: bool = False) -> bool: ...
    @typing.overload
    def solve(self, delassus: DelassusOperatorSparse, g: numpy.ndarray, cones: StdVec_CoulombFrictionCone, R_: numpy.ndarray, primal_solution: numpy.ndarray = None, dual_solution: numpy.ndarray = None, compute_largest_eigen_values: bool = True, stat_record: bool = False) -> bool: ...
    __instance_size__ = 520
    pass

class AngleAxis(Boost.Python.instance):
    """
    AngleAxis representation of a rotation.
    """
    @staticmethod
    def __eq__(arg1: AngleAxis, arg2: AngleAxis) -> bool: 
        """
        __eq__( (AngleAxis)arg1, (AngleAxis)arg2) -> bool
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None :
            Default constructor
        """
    @typing.overload
    def __init__(self, R: numpy.ndarray) -> None: ...
    @typing.overload
    def __init__(self, angle: float, axis: numpy.ndarray) -> None: ...
    @typing.overload
    def __init__(self, copy: AngleAxis) -> None: ...
    @typing.overload
    def __init__(self, quaternion: Quaternion) -> None: ...
    @staticmethod
    @typing.overload
    def __mul__(arg1: AngleAxis, arg2: AngleAxis) -> object: 
        """
        __mul__( (AngleAxis)arg1, (numpy.ndarray)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __mul__(arg1: AngleAxis, arg2: Quaternion) -> object: ...
    @staticmethod
    @typing.overload
    def __mul__(arg1: AngleAxis, arg2: numpy.ndarray) -> object: ...
    @staticmethod
    def __ne__(arg1: AngleAxis, arg2: AngleAxis) -> bool: 
        """
        __ne__( (AngleAxis)arg1, (AngleAxis)arg2) -> bool
        """
    @staticmethod
    def __repr__(arg1: AngleAxis) -> str: 
        """
        __repr__( (AngleAxis)arg1) -> str
        """
    @staticmethod
    def __str__(arg1: AngleAxis) -> str: 
        """
        __str__( (AngleAxis)arg1) -> str
        """
    def fromRotationMatrix(self, rotation_matrix: numpy.ndarray) -> AngleAxis: 
        """
        fromRotationMatrix( (AngleAxis)self, (numpy.ndarray)rotation matrix) -> AngleAxis :
            Sets *this from a 3x3 rotation matrix
        """
    def id(self) -> int: 
        """
        id( (AngleAxis)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def inverse(self) -> AngleAxis: 
        """
        inverse( (AngleAxis)self) -> AngleAxis :
            Return the inverse rotation.
        """
    def isApprox(self, other_: AngleAxis, prec: float) -> bool: 
        """
        isApprox( (AngleAxis)self, (AngleAxis)other [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to other, within the precision determined by prec.
        """
    def matrix(self) -> numpy.ndarray: 
        """
        matrix( (AngleAxis)self) -> numpy.ndarray :
            Returns an equivalent rotation matrix.
        """
    @staticmethod
    def toRotationMatrix(arg1: AngleAxis) -> numpy.ndarray: 
        """
        toRotationMatrix( (AngleAxis)arg1) -> numpy.ndarray :
            Constructs and returns an equivalent rotation matrix.
        """
    @property
    def angle(self) -> float:
        """
        The rotation angle.

        :type: float
        """
    @property
    def axis(self) -> numpy.ndarray:
        """
        The rotation axis.

        :type: numpy.ndarray
        """
    pass

class BaumgarteCorrectorParameters(Boost.Python.instance):
    """
    Paramaters of the Baumgarte Corrector.
    """
    @staticmethod
    def __eq__(arg1: BaumgarteCorrectorParameters, arg2: BaumgarteCorrectorParameters) -> object: 
        """
        __eq__( (BaumgarteCorrectorParameters)arg1, (BaumgarteCorrectorParameters)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, clone: BaumgarteCorrectorParameters) -> object: 
        """
        __init__( (object)self, (int)size) -> None :
            Default constructor.
        """
    @typing.overload
    def __init__(self, size: int) -> None: ...
    @staticmethod
    def __ne__(arg1: BaumgarteCorrectorParameters, arg2: BaumgarteCorrectorParameters) -> object: 
        """
        __ne__( (BaumgarteCorrectorParameters)arg1, (BaumgarteCorrectorParameters)arg2) -> object
        """
    @staticmethod
    def cast(arg1: BaumgarteCorrectorParameters) -> BaumgarteCorrectorParameters: 
        """
        cast( (BaumgarteCorrectorParameters)arg1) -> BaumgarteCorrectorParameters :
            Returns a cast of *this.
        """
    @property
    def Kd(self) -> numpy.ndarray:
        """
        Damping corrector value.

        :type: numpy.ndarray
        """
    @property
    def Kp(self) -> numpy.ndarray:
        """
        Proportional corrector value.

        :type: numpy.ndarray
        """
    pass

class CollisionPair(Boost.Python.instance):
    """
    Pair of ordered index defining a pair of collisions
    """
    def __copy__(self) -> CollisionPair: 
        """
        __copy__( (CollisionPair)self) -> CollisionPair :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> CollisionPair: 
        """
        __deepcopy__( (CollisionPair)self, (dict)memo) -> CollisionPair :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: CollisionPair, arg2: CollisionPair) -> object: 
        """
        __eq__( (CollisionPair)arg1, (CollisionPair)arg2) -> object
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None :
            Empty constructor.
        """
    @typing.overload
    def __init__(self, index1: int, index2: int) -> None: ...
    @staticmethod
    def __ne__(arg1: CollisionPair, arg2: CollisionPair) -> object: 
        """
        __ne__( (CollisionPair)arg1, (CollisionPair)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: CollisionPair) -> object: 
        """
        __repr__( (CollisionPair)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: CollisionPair) -> object: 
        """
        __str__( (CollisionPair)arg1) -> object
        """
    def copy(self) -> CollisionPair: 
        """
        copy( (CollisionPair)self) -> CollisionPair :
            Returns a copy of *this.
        """
    @property
    def first(self) -> int:
        """
        :type: int
        """
    @property
    def second(self) -> int:
        """
        :type: int
        """
    pass

class ContactCholeskyDecomposition(Boost.Python.instance):
    """
    Contact information container for contact dynamic algorithms.
    """
    def __copy__(self) -> ContactCholeskyDecomposition: 
        """
        __copy__( (ContactCholeskyDecomposition)self) -> ContactCholeskyDecomposition :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> ContactCholeskyDecomposition: 
        """
        __deepcopy__( (ContactCholeskyDecomposition)self, (dict)memo) -> ContactCholeskyDecomposition :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: ContactCholeskyDecomposition, arg2: ContactCholeskyDecomposition) -> object: 
        """
        __eq__( (ContactCholeskyDecomposition)arg1, (ContactCholeskyDecomposition)arg2) -> object
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None :
            Default constructor.
        """
    @typing.overload
    def __init__(self, model: Model) -> None: ...
    @typing.overload
    def __init__(self, model: Model, contact_models: StdVec_RigidConstraintModel) -> None: ...
    @staticmethod
    def __ne__(arg1: ContactCholeskyDecomposition, arg2: ContactCholeskyDecomposition) -> object: 
        """
        __ne__( (ContactCholeskyDecomposition)arg1, (ContactCholeskyDecomposition)arg2) -> object
        """
    @typing.overload
    def compute(self, model: Model, data: Data, contact_models: StdVec_RigidConstraintModel, contact_datas: StdVec_RigidConstraintData, mus: numpy.ndarray) -> None: 
        """
        compute( (ContactCholeskyDecomposition)self, (Model)model, (Data)data, (StdVec_RigidConstraintModel)contact_models, (StdVec_RigidConstraintData)contact_datas [, (float)mu=0]) -> None :
            Computes the Cholesky decompostion of the augmented matrix containing the KKT matrix
            related to the system mass matrix and the Jacobians of the contact patches contained in
            the vector of RigidConstraintModel named contact_models. The decomposition is regularized with a factor mu.
        """
    @typing.overload
    def compute(self, model: Model, data: Data, contact_models: StdVec_RigidConstraintModel, contact_datas_: StdVec_RigidConstraintData, mu: float = 0) -> None: ...
    def constraintDim(self) -> int: 
        """
        constraintDim( (ContactCholeskyDecomposition)self) -> int :
            Returns the total dimension of the constraints contained in the Cholesky factorization.
        """
    def copy(self) -> ContactCholeskyDecomposition: 
        """
        copy( (ContactCholeskyDecomposition)self) -> ContactCholeskyDecomposition :
            Returns a copy of *this.
        """
    def getDelassusCholeskyExpression(self) -> DelassusCholeskyExpression: 
        """
        getDelassusCholeskyExpression( (ContactCholeskyDecomposition)self) -> DelassusCholeskyExpression :
            Returns the Cholesky decomposition expression associated to the underlying Delassus matrix.
        """
    def getInverseMassMatrix(self) -> numpy.ndarray: 
        """
        getInverseMassMatrix( (ContactCholeskyDecomposition)self) -> numpy.ndarray :
            Returns the inverse of the Joint Space Inertia Matrix or "mass matrix".
        """
    def getInverseOperationalSpaceInertiaMatrix(self) -> numpy.ndarray: 
        """
        getInverseOperationalSpaceInertiaMatrix( (ContactCholeskyDecomposition)self) -> numpy.ndarray :
            Returns the Inverse of the Operational Space Inertia Matrix resulting from the decomposition.
        """
    @staticmethod
    def getMassMatrixChoeslkyDecomposition(arg1: ContactCholeskyDecomposition, self: Model) -> ContactCholeskyDecomposition: 
        """
        getMassMatrixChoeslkyDecomposition( (ContactCholeskyDecomposition)arg1, (Model)self) -> ContactCholeskyDecomposition :
            Retrieves the Cholesky decomposition of the Mass Matrix contained in the current decomposition.
        """
    def getOperationalSpaceInertiaMatrix(self) -> numpy.ndarray: 
        """
        getOperationalSpaceInertiaMatrix( (ContactCholeskyDecomposition)self) -> numpy.ndarray :
            Returns the Operational Space Inertia Matrix resulting from the decomposition.
        """
    def inverse(self) -> numpy.ndarray: 
        """
        inverse( (ContactCholeskyDecomposition)self) -> numpy.ndarray :
            Returns the inverse matrix resulting from the decomposition.
        """
    def matrix(self) -> numpy.ndarray: 
        """
        matrix( (ContactCholeskyDecomposition)self) -> numpy.ndarray :
            Returns the matrix resulting from the decomposition.
        """
    def numContacts(self) -> int: 
        """
        numContacts( (ContactCholeskyDecomposition)self) -> int :
            Returns the number of contacts associated to this decomposition.
        """
    def size(self) -> int: 
        """
        size( (ContactCholeskyDecomposition)self) -> int :
            Size of the decomposition.
        """
    def solve(self, matrix: numpy.ndarray) -> numpy.ndarray: 
        """
        solve( (ContactCholeskyDecomposition)self, (numpy.ndarray)matrix) -> numpy.ndarray :
            Computes the solution of 
$ A x = b 
$ where self corresponds to the Cholesky decomposition of A.
        """
    @typing.overload
    def updateDamping(self, mu: float) -> None: 
        """
        updateDamping( (ContactCholeskyDecomposition)self, (float)mu) -> None :
            Update the damping term on the upper left block part of the KKT matrix. The damping term should be positive.
        """
    @typing.overload
    def updateDamping(self, mus: numpy.ndarray) -> None: ...
    @property
    def D(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    pass

class CoulombFrictionCone(Boost.Python.instance):
    """
    3d Coulomb friction cone.
    """
    def __copy__(self) -> CoulombFrictionCone: 
        """
        __copy__( (CoulombFrictionCone)self) -> CoulombFrictionCone :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> CoulombFrictionCone: 
        """
        __deepcopy__( (CoulombFrictionCone)self, (dict)memo) -> CoulombFrictionCone :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: CoulombFrictionCone, arg2: CoulombFrictionCone) -> object: 
        """
        __eq__( (CoulombFrictionCone)arg1, (CoulombFrictionCone)arg2) -> object
        """
    @typing.overload
    def __init__(self, mu: float) -> None: 
        """
        __init__( (object)self, (float)mu) -> None :
            Default constructor
        """
    @typing.overload
    def __init__(self, other: CoulombFrictionCone) -> None: ...
    @staticmethod
    def __ne__(arg1: CoulombFrictionCone, arg2: CoulombFrictionCone) -> object: 
        """
        __ne__( (CoulombFrictionCone)arg1, (CoulombFrictionCone)arg2) -> object
        """
    def computeNormalCorrection(self, v: numpy.ndarray) -> numpy.ndarray: 
        """
        computeNormalCorrection( (CoulombFrictionCone)self, (numpy.ndarray)v) -> numpy.ndarray :
            Compute the complementary shift associted to the Coulomb friction cone for complementarity satisfaction in complementary problems.
        """
    def computeRadialProjection(self, f: numpy.ndarray) -> numpy.ndarray: 
        """
        computeRadialProjection( (CoulombFrictionCone)self, (numpy.ndarray)f) -> numpy.ndarray :
            Compute the radial projection associted to the Coulomb friction cone.
        """
    def copy(self) -> CoulombFrictionCone: 
        """
        copy( (CoulombFrictionCone)self) -> CoulombFrictionCone :
            Returns a copy of *this.
        """
    @staticmethod
    def dim() -> int: 
        """
        dim() -> int :
            Returns the dimension of the cone.
        """
    def dual(self) -> DualCoulombFrictionCone: 
        """
        dual( (CoulombFrictionCone)self) -> DualCoulombFrictionCone :
            Returns the dual cone associated to this
        """
    @staticmethod
    def isInside(arg1: CoulombFrictionCone, self: numpy.ndarray, f: float) -> bool: 
        """
        isInside( (CoulombFrictionCone)arg1, (numpy.ndarray)self, (float)f) -> bool :
            Check whether a vector x lies within the cone.
        """
    def project(self, f: numpy.ndarray) -> numpy.ndarray: 
        """
        project( (CoulombFrictionCone)self, (numpy.ndarray)f) -> numpy.ndarray :
            Normal projection of a vector f onto the cone.
        """
    def weightedProject(self, f: numpy.ndarray, R: numpy.ndarray) -> numpy.ndarray: 
        """
        weightedProject( (CoulombFrictionCone)self, (numpy.ndarray)f, (numpy.ndarray)R) -> numpy.ndarray :
            Weighted projection of a vector f onto the cone.
        """
    @property
    def mu(self) -> float:
        """
        Friction coefficient.

        :type: float
        """
    pass

class Data(Boost.Python.instance):
    """
    Articulated rigid body data related to a Model.
    It contains all the data that can be modified by the Pinocchio algorithms.
    """
    def __copy__(self) -> Data: 
        """
        __copy__( (Data)self) -> Data :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> Data: 
        """
        __deepcopy__( (Data)self, (dict)memo) -> Data :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: Data, arg2: Data) -> object: 
        """
        __eq__( (Data)arg1, (Data)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(arg1: Data) -> tuple: 
        """
        __getinitargs__( (Data)arg1) -> tuple
        """
    @staticmethod
    def __getstate__(arg1: Data) -> tuple: 
        """
        __getstate__( (Data)arg1) -> tuple
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None :
            Default constructor.
        """
    @typing.overload
    def __init__(self, model: Model) -> None: ...
    @staticmethod
    def __ne__(arg1: Data, arg2: Data) -> object: 
        """
        __ne__( (Data)arg1, (Data)arg2) -> object
        """
    @staticmethod
    def __setstate__(arg1: Data, arg2: tuple) -> None: 
        """
        __setstate__( (Data)arg1, (tuple)arg2) -> None
        """
    def copy(self) -> Data: 
        """
        copy( (Data)self) -> Data :
            Returns a copy of *this.
        """
    @typing.overload
    def loadFromBinary(self, buffer: StaticBuffer) -> None: 
        """
        loadFromBinary( (Data)self, (str)filename) -> None :
            Loads *this from a binary file.
        """
    @typing.overload
    def loadFromBinary(self, buffer: StreamBuffer) -> None: ...
    @typing.overload
    def loadFromBinary(self, filename: str) -> None: ...
    def loadFromString(self, string: str) -> None: 
        """
        loadFromString( (Data)self, (str)string) -> None :
            Parses from the input string the content of the current object.
        """
    def loadFromText(self, filename: str) -> None: 
        """
        loadFromText( (Data)self, (str)filename) -> None :
            Loads *this from a text file.
        """
    def loadFromXML(self, filename: str, tag_name: str) -> None: 
        """
        loadFromXML( (Data)self, (str)filename, (str)tag_name) -> None :
            Loads *this from a XML file.
        """
    @typing.overload
    def saveToBinary(self, buffer: StaticBuffer) -> None: 
        """
        saveToBinary( (Data)self, (str)filename) -> None :
            Saves *this inside a binary file.
        """
    @typing.overload
    def saveToBinary(self, buffer: StreamBuffer) -> None: ...
    @typing.overload
    def saveToBinary(self, filename: str) -> None: ...
    def saveToString(self) -> str: 
        """
        saveToString( (Data)self) -> str :
            Parses the current object to a string.
        """
    def saveToText(self, filename: str) -> None: 
        """
        saveToText( (Data)self, (str)filename) -> None :
            Saves *this inside a text file.
        """
    @staticmethod
    def saveToXML(arg1: Data, filename: str, tag_name: str) -> None: 
        """
        saveToXML( (Data)arg1, (str)filename, (str)tag_name) -> None :
            Saves *this inside a XML file.
        """
    @property
    def Ag(self) -> numpy.ndarray:
        """
        Centroidal matrix which maps from joint velocity to the centroidal momentum.

        :type: numpy.ndarray
        """
    @property
    def B(self) -> Sequence[FloatMat6]:
        """
        Combined variations of the inertia matrix consistent with Christoffel symbols.

        :type: StdVec_Matrix6
        """
    @property
    def C(self) -> numpy.ndarray:
        """
        The Coriolis C(q,v) matrix such that the Coriolis effects are given by c(q,v) = C(q,v)v

        :type: numpy.ndarray
        """
    @property
    def D(self) -> numpy.ndarray:
        """
        Diagonal of UDUT inertia decomposition

        :type: numpy.ndarray
        """
    @property
    def Fcrb(self) -> Sequence[NDArray[numpy.floating]]:
        """
        Spatial forces set, used in CRBA

        :type: StdVec_Matrix6x
        """
    @property
    def Ig(self) -> Inertia:
        """
        Centroidal Composite Rigid Body Inertia.

        :type: Inertia
        """
    @property
    def Ivx(self) -> Sequence[FloatMat6]:
        """
        Right variation of the inertia matrix.

        :type: StdVec_Matrix6
        """
    @property
    def J(self) -> numpy.ndarray:
        """
        Jacobian of joint placement

        :type: numpy.ndarray
        """
    @property
    def Jcom(self) -> numpy.ndarray:
        """
        Jacobian of center of mass.

        :type: numpy.ndarray
        """
    @property
    def M(self) -> numpy.ndarray:
        """
        The joint space inertia matrix

        :type: numpy.ndarray
        """
    @property
    def Minv(self) -> numpy.ndarray:
        """
        The inverse of the joint space inertia matrix

        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        Joint Inertia square root (upper triangle)

        :type: numpy.ndarray
        """
    @property
    def Yaba(self) -> Sequence[FloatMat6]:
        """
        Articulated Body Inertia of the sub-tree

        :type: StdVec_Matrix6
        """
    @property
    def Ycrb(self) -> Sequence[Inertia]:
        """
        Inertia of the sub-tree composit rigid body

        :type: StdVec_Inertia
        """
    @property
    def a(self) -> Sequence[Motion]:
        """
        Vector of joint accelerations expressed in the local frame of the joint.

        :type: StdVec_Motion
        """
    @property
    def a_gf(self) -> Sequence[Motion]:
        """
        Joint spatial acceleration containing also the contribution of the gravity acceleration

        :type: StdVec_Motion
        """
    @property
    def acom(self) -> Sequence[FloatVec3]:
        """
        CoM acceleration of the subtree starting at joint index i.

        :type: StdVec_Vector3
        """
    @property
    def com(self) -> Sequence[FloatVec3]:
        """
        CoM position of the subtree starting at joint index i.

        :type: StdVec_Vector3
        """
    @property
    def contact_chol(self) -> ContactCholeskyDecomposition:
        """
        Contact Cholesky decomposition.

        :type: ContactCholeskyDecomposition
        """
    @property
    def dAdq(self) -> numpy.ndarray:
        """
        Variation of the spatial acceleration set with respect to the joint configuration.

        :type: numpy.ndarray
        """
    @property
    def dAdv(self) -> numpy.ndarray:
        """
        Variation of the spatial acceleration set with respect to the joint velocity.

        :type: numpy.ndarray
        """
    @property
    def dAg(self) -> numpy.ndarray:
        """
        Time derivative of the centroidal momentum matrix Ag.

        :type: numpy.ndarray
        """
    @property
    def dFda(self) -> numpy.ndarray:
        """
        Variation of the force set with respect to the joint acceleration.

        :type: numpy.ndarray
        """
    @property
    def dFdq(self) -> numpy.ndarray:
        """
        Variation of the force set with respect to the joint configuration.

        :type: numpy.ndarray
        """
    @property
    def dFdv(self) -> numpy.ndarray:
        """
        Variation of the force set with respect to the joint velocity.

        :type: numpy.ndarray
        """
    @property
    def dHdq(self) -> numpy.ndarray:
        """
        Variation of the spatial momenta set with respect to the joint configuration.

        :type: numpy.ndarray
        """
    @property
    def dJ(self) -> numpy.ndarray:
        """
        Time variation of the Jacobian of joint placement (data.J).

        :type: numpy.ndarray
        """
    @property
    def dac_da(self) -> numpy.ndarray:
        """
        Partial derivative of the contact acceleration vector vector with respect to the joint acceleration.

        :type: numpy.ndarray
        """
    @property
    def dac_dq(self) -> numpy.ndarray:
        """
        Partial derivative of the contact acceleration vector with respect to the joint configuration.

        :type: numpy.ndarray
        """
    @property
    def dac_dv(self) -> numpy.ndarray:
        """
        Partial derivative of the contact acceleration vector vector with respect to the joint velocity.

        :type: numpy.ndarray
        """
    @property
    def ddq(self) -> numpy.ndarray:
        """
        Joint accelerations (output of ABA)

        :type: numpy.ndarray
        """
    @property
    def ddq_dq(self) -> numpy.ndarray:
        """
        Partial derivative of the joint acceleration vector with respect to the joint configuration.

        :type: numpy.ndarray
        """
    @property
    def ddq_dtau(self) -> numpy.ndarray:
        """
        Partial derivative of the joint acceleration vector with respect to the joint torque.

        :type: numpy.ndarray
        """
    @property
    def ddq_dv(self) -> numpy.ndarray:
        """
        Partial derivative of the joint acceleration vector with respect to the joint velocity.

        :type: numpy.ndarray
        """
    @property
    def dhg(self) -> Force:
        """
        Centroidal momentum time derivative (expressed in the frame centered at the CoM and aligned with the world frame).

        :type: Force
        """
    @property
    def dlambda_dq(self) -> numpy.ndarray:
        """
        Partial derivative of the contact force vector with respect to the joint configuration.

        :type: numpy.ndarray
        """
    @property
    def dlambda_dtau(self) -> numpy.ndarray:
        """
        Partial derivative of the contact force vector with respect to the torque.

        :type: numpy.ndarray
        """
    @property
    def dlambda_dv(self) -> numpy.ndarray:
        """
        Partial derivative of the contact force vector with respect to the joint velocity.

        :type: numpy.ndarray
        """
    @property
    def dq_after(self) -> numpy.ndarray:
        """
        Generalized velocity after the impact.

        :type: numpy.ndarray
        """
    @property
    def dtau_dq(self) -> numpy.ndarray:
        """
        Partial derivative of the joint torque vector with respect to the joint configuration.

        :type: numpy.ndarray
        """
    @property
    def dtau_dv(self) -> numpy.ndarray:
        """
        Partial derivative of the joint torque vector with respect to the joint velocity.

        :type: numpy.ndarray
        """
    @property
    def dvc_dq(self) -> numpy.ndarray:
        """
        Partial derivative of the constraint velocity vector with respect to the joint configuration.

        :type: numpy.ndarray
        """
    @property
    def f(self) -> Sequence[Force]:
        """
        Vector of body forces expressed in the local frame of the joint.

        :type: StdVec_Force
        """
    @property
    def g(self) -> numpy.ndarray:
        """
        Vector of generalized gravity (dim model.nv).

        :type: numpy.ndarray
        """
    @property
    def h(self) -> Sequence[Force]:
        """
        Vector of spatial momenta expressed in the local frame of the joint.

        :type: StdVec_Force
        """
    @property
    def hg(self) -> Force:
        """
        Centroidal momentum (expressed in the frame centered at the CoM and aligned with the world frame).

        :type: Force
        """
    @property
    def iMf(self) -> Sequence[SE3]:
        """
        Body placement wrt to algorithm end effector.

        :type: StdVec_SE3
        """
    @property
    def impulse_c(self) -> numpy.ndarray:
        """
        Lagrange Multipliers linked to contact impulses

        :type: numpy.ndarray
        """
    @property
    def jointTorqueRegressor(self) -> numpy.ndarray:
        """
        Joint torque regressor.

        :type: numpy.ndarray
        """
    @property
    def joints(self) -> Sequence[JointData]:
        """
        Vector of JointData associated to each JointModel stored in the related model.

        :type: StdVec_JointDataVector
        """
    @property
    def kineticEnergyRegressor(self) -> numpy.ndarray:
        """
        Kinetic energy regressor.

        :type: numpy.ndarray
        """
    @property
    def kinetic_energy(self) -> float:
        """
        Kinetic energy in [J] computed by computeKineticEnergy

        :type: float
        """
    @property
    def lambda_c(self) -> numpy.ndarray:
        """
        Lagrange Multipliers linked to contact forces

        :type: numpy.ndarray
        """
    @property
    def lambda_c_prox(self) -> numpy.ndarray:
        """
        Proximal Lagrange Multipliers used in the computation of the Forward Dynamics computations.

        :type: numpy.ndarray
        """
    @property
    def lastChild(self) -> Sequence[int]:
        """
        Index of the last child (for CRBA)

        :type: StdVec_int
        """
    @property
    def liMi(self) -> Sequence[SE3]:
        """
        Body relative placement (wrt parent)

        :type: StdVec_SE3
        """
    @property
    def mass(self) -> Sequence[float]:
        """
        Mass of the subtree starting at joint index i.

        :type: StdVec_Scalar
        """
    @property
    def mechanical_energy(self) -> float:
        """
        Mechanical energy in [J] of the system computed by computeMechanicalEnergy

        :type: float
        """
    @property
    def nle(self) -> numpy.ndarray:
        """
        Non Linear Effects (output of nle algorithm)

        :type: numpy.ndarray
        """
    @property
    def nvSubtree(self) -> Sequence[int]:
        """
        Dimension of the subtree motion space (for CRBA)

        :type: StdVec_int
        """
    @property
    def nvSubtree_fromRow(self) -> Sequence[int]:
        """
        Subtree of the current row index (used in Cholesky)

        :type: StdVec_int
        """
    @property
    def oK(self) -> Sequence[FloatMat6]:
        """
        Inverse articulated inertia.

        :type: StdVec_Matrix6
        """
    @property
    def oL(self) -> Sequence[FloatMat6]:
        """
        Acceleration propagator.

        :type: StdVec_Matrix6
        """
    @property
    def oMf(self) -> Sequence[SE3]:
        """
        frames absolute placement (wrt world)

        :type: StdVec_SE3
        """
    @property
    def oMi(self) -> Sequence[SE3]:
        """
        Body absolute placement (wrt world)

        :type: StdVec_SE3
        """
    @property
    def oYaba(self) -> Sequence[FloatMat6]:
        """
        Articulated Body Inertia of the sub-tree expressed in the WORLD coordinate system.

        :type: StdVec_Matrix6
        """
    @property
    def oYcrb(self) -> Sequence[Inertia]:
        """
        Composite Rigid Body Inertia of the sub-tree expressed in the WORLD coordinate system.

        :type: StdVec_Inertia
        """
    @property
    def oa(self) -> Sequence[Motion]:
        """
        Joint spatial acceleration expressed at the origin of the world frame.

        :type: StdVec_Motion
        """
    @property
    def oa_gf(self) -> Sequence[Motion]:
        """
        Joint spatial acceleration containing also the contribution of the gravity acceleration, but expressed at the origin of the world frame.

        :type: StdVec_Motion
        """
    @property
    def of(self) -> Sequence[Force]:
        """
        Vector of body forces expressed at the origin of the world.

        :type: StdVec_Force
        """
    @property
    def of_augmented(self) -> Sequence[Force]:
        """
        Vector of body forces expressed at the origin of the world, in the context of lagrangian formulation

        :type: StdVec_Force
        """
    @property
    def oh(self) -> Sequence[Force]:
        """
        Vector of spatial momenta expressed at the origin of the world.

        :type: StdVec_Force
        """
    @property
    def osim(self) -> numpy.ndarray:
        """
        Operational space inertia matrix.

        :type: numpy.ndarray
        """
    @property
    def ov(self) -> Sequence[Motion]:
        """
        Vector of joint velocities expressed at the origin of the world.

        :type: StdVec_Motion
        """
    @property
    def parents_fromRow(self) -> Sequence[int]:
        """
        First previous non-zero row in M (used in Cholesky)

        :type: StdVec_int
        """
    @property
    def potentialEnergyRegressor(self) -> numpy.ndarray:
        """
        Potential energy regressor.

        :type: numpy.ndarray
        """
    @property
    def potential_energy(self) -> float:
        """
        Potential energy in [J] computed by computePotentialEnergy

        :type: float
        """
    @property
    def primal_dual_contact_solution(self) -> numpy.ndarray:
        """
        Right hand side vector when solving the contact dynamics KKT problem.

        :type: numpy.ndarray
        """
    @property
    def primal_rhs_contact(self) -> numpy.ndarray:
        """
        Primal RHS in contact dynamic equations.

        :type: numpy.ndarray
        """
    @property
    def staticRegressor(self) -> numpy.ndarray:
        """
        Static regressor.

        :type: numpy.ndarray
        """
    @property
    def tau(self) -> numpy.ndarray:
        """
        Joint torques (output of RNEA)

        :type: numpy.ndarray
        """
    @property
    def v(self) -> Sequence[Motion]:
        """
        Vector of joint velocities expressed in the local frame of the joint.

        :type: StdVec_Motion
        """
    @property
    def vcom(self) -> Sequence[FloatVec3]:
        """
        CoM velocity of the subtree starting at joint index i.

        :type: StdVec_Vector3
        """
    @property
    def vxI(self) -> Sequence[FloatMat6]:
        """
        Left variation of the inertia matrix.

        :type: StdVec_Matrix6
        """
    __getstate_manages_dict__ = True
    __safe_for_unpickling__ = True
    pass

class DelassusCholeskyExpression(Boost.Python.instance):
    """
    Delassus Cholesky expression associated to a given ContactCholeskyDecomposition object.
    """
    def __init__(self, cholesky_decomposition: ContactCholeskyDecomposition) -> None: 
        """
        __init__( (object)self, (ContactCholeskyDecomposition)cholesky_decomposition) -> None :
            Build from a given ContactCholeskyDecomposition object.
        """
    def __matmul__(self, other: numpy.ndarray) -> numpy.ndarray: 
        """
        __matmul__( (DelassusCholeskyExpression)self, (numpy.ndarray)other) -> numpy.ndarray :
            Matrix multiplication between self and another matrix. Returns the result of Delassus * matrix.
        """
    @staticmethod
    def __mul__(arg1: DelassusCholeskyExpression, arg2: numpy.ndarray) -> object: 
        """
        __mul__( (DelassusCholeskyExpression)arg1, (numpy.ndarray)arg2) -> object
        """
    def cholesky(self) -> ContactCholeskyDecomposition: 
        """
        cholesky( (DelassusCholeskyExpression)self) -> ContactCholeskyDecomposition :
            Returns the Constraint Cholesky decomposition associated to this DelassusCholeskyExpression.
        """
    def cols(self) -> int: 
        """
        cols( (DelassusCholeskyExpression)self) -> int :
            Returns the number of columns.
        """
    def computeLargestEigenValue(self, reset: bool = True, max_it: int = 10, prec: float = 1e-08) -> float: 
        """
        computeLargestEigenValue( (DelassusCholeskyExpression)self [, (bool)reset=True [, (int)max_it=10 [, (float)prec=1e-08]]]) -> float :
            Compute the largest eigenvalue associated to the underlying Delassus matrix.
        """
    def computeLowestEigenValue(self, reset: bool = True, compute_largest: bool = True, max_it: int = 10, prec: float = 1e-08) -> float: 
        """
        computeLowestEigenValue( (DelassusCholeskyExpression)self [, (bool)reset=True [, (bool)compute_largest=True [, (int)max_it=10 [, (float)prec=1e-08]]]]) -> float :
            Compute the lowest eigenvalue associated to the underlying Delassus matrix.
        """
    def inverse(self) -> numpy.ndarray: 
        """
        inverse( (DelassusCholeskyExpression)self) -> numpy.ndarray :
            Returns the inverse of the Delassus expression as a dense matrix.
        """
    def matrix(self) -> numpy.ndarray: 
        """
        matrix( (DelassusCholeskyExpression)self) -> numpy.ndarray :
            Returns the Delassus expression as a dense matrix.
        """
    def rows(self) -> int: 
        """
        rows( (DelassusCholeskyExpression)self) -> int :
            Returns the number of rows.
        """
    def size(self) -> int: 
        """
        size( (DelassusCholeskyExpression)self) -> int :
            Returns the size of the decomposition.
        """
    def solve(self, mat: numpy.ndarray) -> numpy.ndarray: 
        """
        solve( (DelassusCholeskyExpression)self, (numpy.ndarray)mat) -> numpy.ndarray :
            Returns the solution x of Delassus * x = mat using the current decomposition of the Delassus matrix.
        """
    @typing.overload
    def updateDamping(self, mu: float) -> None: 
        """
        updateDamping( (DelassusCholeskyExpression)self, (float)mu) -> None :
            Add a damping term to the diagonal of the Delassus matrix. The damping term should be positive.
        """
    @typing.overload
    def updateDamping(self, mus: numpy.ndarray) -> None: ...
    pass

class DelassusOperatorDense(Boost.Python.instance):
    """
    Delassus Cholesky dense operator from a dense matrix.
    """
    def __init__(self, matrix: numpy.ndarray) -> None: 
        """
        __init__( (object)self, (numpy.ndarray)matrix) -> None :
            Build from a given dense matrix
        """
    def __matmul__(self, other: numpy.ndarray) -> numpy.ndarray: 
        """
        __matmul__( (DelassusOperatorDense)self, (numpy.ndarray)other) -> numpy.ndarray :
            Matrix multiplication between self and another matrix. Returns the result of Delassus * matrix.
        """
    @staticmethod
    def __mul__(arg1: DelassusOperatorDense, arg2: numpy.ndarray) -> object: 
        """
        __mul__( (DelassusOperatorDense)arg1, (numpy.ndarray)arg2) -> object
        """
    def cols(self) -> int: 
        """
        cols( (DelassusOperatorDense)self) -> int :
            Returns the number of columns.
        """
    def computeLargestEigenValue(self, reset: bool = True, max_it: int = 10, prec: float = 1e-08) -> float: 
        """
        computeLargestEigenValue( (DelassusOperatorDense)self [, (bool)reset=True [, (int)max_it=10 [, (float)prec=1e-08]]]) -> float :
            Compute the largest eigenvalue associated to the underlying Delassus matrix.
        """
    def computeLowestEigenValue(self, reset: bool = True, compute_largest: bool = True, max_it: int = 10, prec: float = 1e-08) -> float: 
        """
        computeLowestEigenValue( (DelassusOperatorDense)self [, (bool)reset=True [, (bool)compute_largest=True [, (int)max_it=10 [, (float)prec=1e-08]]]]) -> float :
            Compute the lowest eigenvalue associated to the underlying Delassus matrix.
        """
    def inverse(self) -> numpy.ndarray: 
        """
        inverse( (DelassusOperatorDense)self) -> numpy.ndarray :
            Returns the inverse of the Delassus expression as a dense matrix.
        """
    def matrix(self) -> numpy.ndarray: 
        """
        matrix( (DelassusOperatorDense)self) -> numpy.ndarray :
            Returns the Delassus expression as a dense matrix.
        """
    def rows(self) -> int: 
        """
        rows( (DelassusOperatorDense)self) -> int :
            Returns the number of rows.
        """
    def size(self) -> int: 
        """
        size( (DelassusOperatorDense)self) -> int :
            Returns the size of the decomposition.
        """
    def solve(self, mat: numpy.ndarray) -> numpy.ndarray: 
        """
        solve( (DelassusOperatorDense)self, (numpy.ndarray)mat) -> numpy.ndarray :
            Returns the solution x of Delassus * x = mat using the current decomposition of the Delassus matrix.
        """
    @typing.overload
    def updateDamping(self, mu: float) -> None: 
        """
        updateDamping( (DelassusOperatorDense)self, (float)mu) -> None :
            Add a damping term to the diagonal of the Delassus matrix. The damping term should be positive.
        """
    @typing.overload
    def updateDamping(self, mus: numpy.ndarray) -> None: ...
    pass

class DelassusOperatorSparse(Boost.Python.instance):
    """
    Delassus Cholesky sparse operator from a sparse matrix.
    """
    def __init__(self, matrix: csc_matrix) -> None: 
        """
        __init__( (object)self, (csc_matrix)matrix) -> None :
            Build from a given sparse matrix
        """
    def __matmul__(self, other: numpy.ndarray) -> numpy.ndarray: 
        """
        __matmul__( (DelassusOperatorSparse)self, (numpy.ndarray)other) -> numpy.ndarray :
            Matrix multiplication between self and another matrix. Returns the result of Delassus * matrix.
        """
    @staticmethod
    def __mul__(arg1: DelassusOperatorSparse, arg2: numpy.ndarray) -> object: 
        """
        __mul__( (DelassusOperatorSparse)arg1, (numpy.ndarray)arg2) -> object
        """
    def cols(self) -> int: 
        """
        cols( (DelassusOperatorSparse)self) -> int :
            Returns the number of columns.
        """
    def computeLargestEigenValue(self, reset: bool = True, max_it: int = 10, prec: float = 1e-08) -> float: 
        """
        computeLargestEigenValue( (DelassusOperatorSparse)self [, (bool)reset=True [, (int)max_it=10 [, (float)prec=1e-08]]]) -> float :
            Compute the largest eigenvalue associated to the underlying Delassus matrix.
        """
    def computeLowestEigenValue(self, reset: bool = True, compute_largest: bool = True, max_it: int = 10, prec: float = 1e-08) -> float: 
        """
        computeLowestEigenValue( (DelassusOperatorSparse)self [, (bool)reset=True [, (bool)compute_largest=True [, (int)max_it=10 [, (float)prec=1e-08]]]]) -> float :
            Compute the lowest eigenvalue associated to the underlying Delassus matrix.
        """
    def inverse(self) -> csc_matrix: 
        """
        inverse( (DelassusOperatorSparse)self) -> csc_matrix :
            Returns the inverse of the Delassus expression as a dense matrix.
        """
    def matrix(self) -> csc_matrix: 
        """
        matrix( (DelassusOperatorSparse)self) -> csc_matrix :
            Returns the Delassus expression as a dense matrix.
        """
    def rows(self) -> int: 
        """
        rows( (DelassusOperatorSparse)self) -> int :
            Returns the number of rows.
        """
    def size(self) -> int: 
        """
        size( (DelassusOperatorSparse)self) -> int :
            Returns the size of the decomposition.
        """
    def solve(self, mat: numpy.ndarray) -> numpy.ndarray: 
        """
        solve( (DelassusOperatorSparse)self, (numpy.ndarray)mat) -> numpy.ndarray :
            Returns the solution x of Delassus * x = mat using the current decomposition of the Delassus matrix.
        """
    @typing.overload
    def updateDamping(self, mu: float) -> None: 
        """
        updateDamping( (DelassusOperatorSparse)self, (float)mu) -> None :
            Add a damping term to the diagonal of the Delassus matrix. The damping term should be positive.
        """
    @typing.overload
    def updateDamping(self, mus: numpy.ndarray) -> None: ...
    pass

class DualCoulombFrictionCone(Boost.Python.instance):
    """
    Dual cone of the 3d Coulomb friction cone.
    """
    def __copy__(self) -> DualCoulombFrictionCone: 
        """
        __copy__( (DualCoulombFrictionCone)self) -> DualCoulombFrictionCone :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> DualCoulombFrictionCone: 
        """
        __deepcopy__( (DualCoulombFrictionCone)self, (dict)memo) -> DualCoulombFrictionCone :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: DualCoulombFrictionCone, arg2: DualCoulombFrictionCone) -> object: 
        """
        __eq__( (DualCoulombFrictionCone)arg1, (DualCoulombFrictionCone)arg2) -> object
        """
    @typing.overload
    def __init__(self, mu: float) -> None: 
        """
        __init__( (object)self, (float)mu) -> None :
            Default constructor
        """
    @typing.overload
    def __init__(self, other: DualCoulombFrictionCone) -> None: ...
    @staticmethod
    def __ne__(arg1: DualCoulombFrictionCone, arg2: DualCoulombFrictionCone) -> object: 
        """
        __ne__( (DualCoulombFrictionCone)arg1, (DualCoulombFrictionCone)arg2) -> object
        """
    def copy(self) -> DualCoulombFrictionCone: 
        """
        copy( (DualCoulombFrictionCone)self) -> DualCoulombFrictionCone :
            Returns a copy of *this.
        """
    @staticmethod
    def dim() -> int: 
        """
        dim() -> int :
            Returns the dimension of the cone.
        """
    def dual(self) -> CoulombFrictionCone: 
        """
        dual( (DualCoulombFrictionCone)self) -> CoulombFrictionCone :
            Returns the dual cone associated to this.
        """
    @staticmethod
    def isInside(arg1: DualCoulombFrictionCone, self: numpy.ndarray, v: float) -> bool: 
        """
        isInside( (DualCoulombFrictionCone)arg1, (numpy.ndarray)self, (float)v) -> bool :
            Check whether a vector x lies within the cone.
        """
    def project(self, v: numpy.ndarray) -> numpy.ndarray: 
        """
        project( (DualCoulombFrictionCone)self, (numpy.ndarray)v) -> numpy.ndarray :
            Project a vector v onto the cone.
        """
    @property
    def mu(self) -> float:
        """
        Friction coefficient.

        :type: float
        """
    pass

class Exception(Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object, arg2: str) -> None: 
        """
        __init__( (object)arg1, (str)arg2) -> None
        """
    @property
    def message(self) -> str:
        """
        :type: str
        """
    __instance_size__ = 72
    pass

class Force(Boost.Python.instance):
    """
    Force vectors, in se3* == F^6.

    Supported operations ...
    """
    @staticmethod
    def Random() -> Force: 
        """
        Random() -> Force :
            Returns a random Force.
        """
    @staticmethod
    def Zero() -> Force: 
        """
        Zero() -> Force :
            Returns a zero Force.
        """
    @staticmethod
    def __add__(arg1: Force, arg2: Force) -> object: 
        """
        __add__( (Force)arg1, (Force)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __array__(arg1: Force) -> object: 
        """
        __array__( (Force)arg1) -> object
        """
    @typing.overload
    def __array__(self, dtype: object = None, copy: object = None) -> object: ...
    def __copy__(self) -> Force: 
        """
        __copy__( (Force)self) -> Force :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> Force: 
        """
        __deepcopy__( (Force)self, (dict)memo) -> Force :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: Force, arg2: Force) -> object: 
        """
        __eq__( (Force)arg1, (Force)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(arg1: Force) -> tuple: 
        """
        __getinitargs__( (Force)arg1) -> tuple
        """
    @staticmethod
    def __iadd__(arg1: object, arg2: Force) -> object: 
        """
        __iadd__( (object)arg1, (Force)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, clone: Force) -> object: 
        """
        __init__( (object)self) -> None :
            Default constructor
        """
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None: ...
    @typing.overload
    def __init__(self, clone: Force) -> None: ...
    @typing.overload
    def __init__(self, linear: numpy.ndarray, angular: numpy.ndarray) -> None: ...
    @staticmethod
    def __isub__(arg1: object, arg2: Force) -> object: 
        """
        __isub__( (object)arg1, (Force)arg2) -> object
        """
    @staticmethod
    def __mul__(arg1: Force, arg2: float) -> object: 
        """
        __mul__( (Force)arg1, (float)arg2) -> object
        """
    @staticmethod
    def __ne__(arg1: Force, arg2: Force) -> object: 
        """
        __ne__( (Force)arg1, (Force)arg2) -> object
        """
    @staticmethod
    def __neg__(arg1: Force) -> object: 
        """
        __neg__( (Force)arg1) -> object
        """
    @staticmethod
    def __repr__(arg1: Force) -> object: 
        """
        __repr__( (Force)arg1) -> object
        """
    @staticmethod
    def __rmul__(arg1: Force, arg2: float) -> object: 
        """
        __rmul__( (Force)arg1, (float)arg2) -> object
        """
    @staticmethod
    def __str__(arg1: Force) -> object: 
        """
        __str__( (Force)arg1) -> object
        """
    @staticmethod
    def __sub__(arg1: Force, arg2: Force) -> object: 
        """
        __sub__( (Force)arg1, (Force)arg2) -> object
        """
    @staticmethod
    def __truediv__(arg1: Force, arg2: float) -> object: 
        """
        __truediv__( (Force)arg1, (float)arg2) -> object
        """
    @staticmethod
    def cast(arg1: Force) -> Force: 
        """
        cast( (Force)arg1) -> Force :
            Returns a cast of *this.
        """
    def copy(self) -> Force: 
        """
        copy( (Force)self) -> Force :
            Returns a copy of *this.
        """
    def dot(self, m: object) -> float: 
        """
        dot( (Force)self, (object)m) -> float :
            Dot product between *this and a Motion m.
        """
    def isApprox(self, other_: Force, prec: float = 1e-12) -> bool: 
        """
        isApprox( (Force)self, (Force)other [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to other, within the precision given by prec.
        """
    def isZero(self, prec: float = 1e-12) -> bool: 
        """
        isZero( (Force)self [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to the zero Force, within the precision given by prec.
        """
    def se3Action(self, M: SE3) -> Force: 
        """
        se3Action( (Force)self, (SE3)M) -> Force :
            Returns the result of the dual action of M on *this.
        """
    def se3ActionInverse(self, M: SE3) -> Force: 
        """
        se3ActionInverse( (Force)self, (SE3)M) -> Force :
            Returns the result of the dual action of the inverse of M on *this.
        """
    def setRandom(self) -> None: 
        """
        setRandom( (Force)self) -> None :
            Set the linear and angular components of *this to random values.
        """
    def setZero(self) -> None: 
        """
        setZero( (Force)self) -> None :
            Set the linear and angular components of *this to zero.
        """
    @property
    def angular(self) -> numpy.ndarray:
        """
        Angular part of a *this, corresponding to the angular velocity in case of a Spatial velocity.

        :type: numpy.ndarray
        """
    @property
    def linear(self) -> numpy.ndarray:
        """
        Linear part of a *this, corresponding to the linear velocity in case of a Spatial velocity.

        :type: numpy.ndarray
        """
    @property
    def np(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def vector(self) -> numpy.ndarray:
        """
        Returns the components of *this as a 6d vector.

        :type: numpy.ndarray
        """
    __safe_for_unpickling__ = True
    pass

class Frame(Boost.Python.instance):
    """
    A Plucker coordinate frame related to a parent joint inside a kinematic tree.
    """
    def __copy__(self) -> Frame: 
        """
        __copy__( (Frame)self) -> Frame :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> Frame: 
        """
        __deepcopy__( (Frame)self, (dict)memo) -> Frame :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: Frame, arg2: Frame) -> object: 
        """
        __eq__( (Frame)arg1, (Frame)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(arg1: Frame) -> tuple: 
        """
        __getinitargs__( (Frame)arg1) -> tuple
        """
    @staticmethod
    def __getstate__(arg1: Frame) -> tuple: 
        """
        __getstate__( (Frame)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, clone: Frame) -> object: 
        """
        __init__( (object)self) -> None :
            Default constructor
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, name: str, parent_joint: int, parent_frame: int, placement: SE3, type_: FrameType, inertia: Inertia) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, name: str, parent_joint: int, placement: SE3, type_: FrameType, inertia: Inertia) -> None: ...
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, clone: Frame) -> None: ...
    @typing.overload
    def __init__(self, other: Frame) -> None: ...
    @staticmethod
    def __ne__(arg1: Frame, arg2: Frame) -> object: 
        """
        __ne__( (Frame)arg1, (Frame)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: Frame) -> object: 
        """
        __repr__( (Frame)arg1) -> object
        """
    @staticmethod
    def __setstate__(arg1: Frame, arg2: tuple) -> None: 
        """
        __setstate__( (Frame)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def __str__(arg1: Frame) -> object: 
        """
        __str__( (Frame)arg1) -> object
        """
    @staticmethod
    def cast(arg1: Frame) -> Frame: 
        """
        cast( (Frame)arg1) -> Frame :
            Returns a cast of *this.
        """
    def copy(self) -> Frame: 
        """
        copy( (Frame)self) -> Frame :
            Returns a copy of *this.
        """
    @property
    def inertia(self) -> Inertia:
        """
        Inertia information attached to the frame.

        :type: Inertia
        """
    @property
    def name(self) -> str:
        """
        name of the frame

        :type: str
        """
    @property
    def parent(self) -> int:
        """
        See parentJoint property.

        :type: int
        """
    @property
    def parentFrame(self) -> int:
        """
        Index of the parent frame

        :type: int
        """
    @property
    def parentJoint(self) -> int:
        """
        Index of the parent joint

        :type: int
        """
    @property
    def placement(self) -> SE3:
        """
        placement in the parent joint local frame

        :type: SE3
        """
    @property
    def previousFrame(self) -> int:
        """
        See parentFrame property.

        :type: int
        """
    @property
    def type(self) -> FrameType:
        """
        type of the frame

        :type: FrameType
        """
    __getstate_manages_dict__ = True
    __safe_for_unpickling__ = True
    pass

class GeometryData(Boost.Python.instance):
    """
    Geometry data linked to a Geometry Model and a Data struct.
    """
    def __address__(self) -> int: 
        """
        __address__( (GeometryModel)self) -> int :
            Returns the address of the underlying C++ object.
        """
    def __copy__(self) -> GeometryData: 
        """
        __copy__( (GeometryData)self) -> GeometryData :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> GeometryData: 
        """
        __deepcopy__( (GeometryData)self, (dict)memo) -> GeometryData :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: GeometryData, arg2: GeometryData) -> object: 
        """
        __eq__( (GeometryData)arg1, (GeometryData)arg2) -> object
        """
    def __init__(self, geometry_model: GeometryModel) -> None: 
        """
        __init__( (object)self, (GeometryModel)geometry_model) -> None :
            Default constructor from a given GeometryModel.
        """
    @staticmethod
    def __ne__(arg1: GeometryData, arg2: GeometryData) -> object: 
        """
        __ne__( (GeometryData)arg1, (GeometryData)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: GeometryData) -> object: 
        """
        __repr__( (GeometryData)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: GeometryData) -> object: 
        """
        __str__( (GeometryData)arg1) -> object
        """
    def activateCollisionPair(self, pair_id: int) -> None: 
        """
        activateCollisionPair( (GeometryData)self, (int)pair_id) -> None :
            Activate the collsion pair pair_id in geomModel.collisionPairs if it exists.
            note: Only active pairs are check for collision and distance computations.
        """
    def copy(self) -> GeometryData: 
        """
        copy( (GeometryData)self) -> GeometryData :
            Returns a copy of *this.
        """
    def deactivateAllCollisionPairs(self) -> None: 
        """
        deactivateAllCollisionPairs( (GeometryData)self) -> None :
            Deactivate all collision pairs.
        """
    def deactivateCollisionPair(self, pair_id: int) -> None: 
        """
        deactivateCollisionPair( (GeometryData)self, (int)pair_id) -> None :
            Deactivate the collsion pair pair_id in geomModel.collisionPairs if it exists.
        """
    def fillInnerOuterObjectMaps(self, geometry_model: GeometryModel) -> None: 
        """
        fillInnerOuterObjectMaps( (GeometryData)self, (GeometryModel)geometry_model) -> None :
            Fill inner and outer objects maps
        """
    @typing.overload
    def loadFromBinary(self, buffer: StaticBuffer) -> None: 
        """
        loadFromBinary( (GeometryData)self, (str)filename) -> None :
            Loads *this from a binary file.
        """
    @typing.overload
    def loadFromBinary(self, buffer: StreamBuffer) -> None: ...
    @typing.overload
    def loadFromBinary(self, filename: str) -> None: ...
    def loadFromString(self, string: str) -> None: 
        """
        loadFromString( (GeometryData)self, (str)string) -> None :
            Parses from the input string the content of the current object.
        """
    def loadFromText(self, filename: str) -> None: 
        """
        loadFromText( (GeometryData)self, (str)filename) -> None :
            Loads *this from a text file.
        """
    def loadFromXML(self, filename: str, tag_name: str) -> None: 
        """
        loadFromXML( (GeometryData)self, (str)filename, (str)tag_name) -> None :
            Loads *this from a XML file.
        """
    @typing.overload
    def saveToBinary(self, buffer: StaticBuffer) -> None: 
        """
        saveToBinary( (GeometryData)self, (str)filename) -> None :
            Saves *this inside a binary file.
        """
    @typing.overload
    def saveToBinary(self, buffer: StreamBuffer) -> None: ...
    @typing.overload
    def saveToBinary(self, filename: str) -> None: ...
    def saveToString(self) -> str: 
        """
        saveToString( (GeometryData)self) -> str :
            Parses the current object to a string.
        """
    def saveToText(self, filename: str) -> None: 
        """
        saveToText( (GeometryData)self, (str)filename) -> None :
            Saves *this inside a text file.
        """
    @staticmethod
    def saveToXML(arg1: GeometryData, filename: str, tag_name: str) -> None: 
        """
        saveToXML( (GeometryData)arg1, (str)filename, (str)tag_name) -> None :
            Saves *this inside a XML file.
        """
    def setActiveCollisionPairs(self, geometry_model: GeometryModel, collision_map_: numpy.ndarray, upper: bool = True) -> None: 
        """
        setActiveCollisionPairs( (GeometryData)self, (GeometryModel)geometry_model, (numpy.ndarray)collision_map [, (bool)upper=True]) -> None :
            Set the collision pair association from a given input array.
            Each entry of the input matrix defines the activation of a given collision pair.
        """
    def setGeometryCollisionStatus(self, geom_model: GeometryModel, geom_id: int, enable_collision: bool) -> None: 
        """
        setGeometryCollisionStatus( (GeometryData)self, (GeometryModel)geom_model, (int)geom_id, (bool)enable_collision) -> None :
            Enable or disable collision for the given geometry given by its geometry id with all the other geometries registered in the list of collision pairs.
        """
    @property
    def activeCollisionPairs(self) -> Sequence[bool]:
        """
        Vector of active CollisionPairs

        :type: StdVec_Bool
        """
    @property
    def oMg(self) -> Sequence[SE3]:
        """
        Vector of collision objects placement relative to the world frame.
        note: These quantities have to be updated by calling updateGeometryPlacements.

        :type: StdVec_SE3
        """
    pass

class GeometryModel(Boost.Python.instance):
    """
    Geometry model containing the collision or visual geometries associated to a model.
    """
    def __address__(self) -> int: 
        """
        __address__( (GeometryModel)self) -> int :
            Returns the address of the underlying C++ object.
        """
    def __copy__(self) -> GeometryModel: 
        """
        __copy__( (GeometryModel)self) -> GeometryModel :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> GeometryModel: 
        """
        __deepcopy__( (GeometryModel)self, (dict)memo) -> GeometryModel :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: GeometryModel, arg2: GeometryModel) -> object: 
        """
        __eq__( (GeometryModel)arg1, (GeometryModel)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(arg1: GeometryModel) -> tuple: 
        """
        __getinitargs__( (GeometryModel)arg1) -> tuple
        """
    @staticmethod
    def __getstate__(arg1: GeometryModel) -> tuple: 
        """
        __getstate__( (GeometryModel)arg1) -> tuple
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None :
            Default constructor
        """
    @typing.overload
    def __init__(self, other: GeometryModel) -> None: ...
    @staticmethod
    def __ne__(arg1: GeometryModel, arg2: GeometryModel) -> object: 
        """
        __ne__( (GeometryModel)arg1, (GeometryModel)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: GeometryModel) -> object: 
        """
        __repr__( (GeometryModel)arg1) -> object
        """
    @staticmethod
    def __setstate__(arg1: GeometryModel, arg2: tuple) -> None: 
        """
        __setstate__( (GeometryModel)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def __str__(arg1: GeometryModel) -> object: 
        """
        __str__( (GeometryModel)arg1) -> object
        """
    @staticmethod
    def addAllCollisionPairs(arg1: GeometryModel) -> None: 
        """
        addAllCollisionPairs( (GeometryModel)arg1) -> None :
            Add all collision pairs.
            note : collision pairs between geometries having the same parent joint are not added.
        """
    def addCollisionPair(self, collision_pair: CollisionPair) -> None: 
        """
        addCollisionPair( (GeometryModel)self, (CollisionPair)collision_pair) -> None :
            Add a collision pair given by the index of the two collision objects.
        """
    @typing.overload
    def addGeometryObject(self, geometry_object: GeometryObject) -> int: 
        """
        addGeometryObject( (GeometryModel)self, (GeometryObject)geometry_object) -> int :
            Add a GeometryObject to a GeometryModel.
            Parameters
            	geometry_object : a GeometryObject
            
        """
    @typing.overload
    def addGeometryObject(self, geometry_object: GeometryObject, model: Model) -> int: ...
    def clone(self) -> GeometryModel: 
        """
        clone( (GeometryModel)self) -> GeometryModel :
            Create a deep copy of *this.
        """
    def copy(self) -> GeometryModel: 
        """
        copy( (GeometryModel)self) -> GeometryModel :
            Returns a copy of *this.
        """
    def createData(self) -> GeometryData: 
        """
        createData( (GeometryModel)self) -> GeometryData :
            Create a GeometryData associated to the current model.
        """
    def existCollisionPair(self, collision_pair: CollisionPair) -> bool: 
        """
        existCollisionPair( (GeometryModel)self, (CollisionPair)collision_pair) -> bool :
            Check if a collision pair exists.
        """
    def existGeometryName(self, name: str) -> bool: 
        """
        existGeometryName( (GeometryModel)self, (str)name) -> bool :
            Checks if a GeometryObject  given by its name exists.
        """
    def findCollisionPair(self, collision_pair: CollisionPair) -> int: 
        """
        findCollisionPair( (GeometryModel)self, (CollisionPair)collision_pair) -> int :
            Return the index of a collision pair.
        """
    def getGeometryId(self, name: str) -> int: 
        """
        getGeometryId( (GeometryModel)self, (str)name) -> int :
            Returns the index of a GeometryObject given by its name.
        """
    @typing.overload
    def loadFromBinary(self, buffer: StaticBuffer) -> None: 
        """
        loadFromBinary( (GeometryModel)self, (str)filename) -> None :
            Loads *this from a binary file.
        """
    @typing.overload
    def loadFromBinary(self, buffer: StreamBuffer) -> None: ...
    @typing.overload
    def loadFromBinary(self, filename: str) -> None: ...
    def loadFromString(self, string: str) -> None: 
        """
        loadFromString( (GeometryModel)self, (str)string) -> None :
            Parses from the input string the content of the current object.
        """
    def loadFromText(self, filename: str) -> None: 
        """
        loadFromText( (GeometryModel)self, (str)filename) -> None :
            Loads *this from a text file.
        """
    def loadFromXML(self, filename: str, tag_name: str) -> None: 
        """
        loadFromXML( (GeometryModel)self, (str)filename, (str)tag_name) -> None :
            Loads *this from a XML file.
        """
    @staticmethod
    def removeAllCollisionPairs(arg1: GeometryModel) -> None: 
        """
        removeAllCollisionPairs( (GeometryModel)arg1) -> None :
            Remove all collision pairs.
        """
    def removeCollisionPair(self, collision_pair: CollisionPair) -> None: 
        """
        removeCollisionPair( (GeometryModel)self, (CollisionPair)collision_pair) -> None :
            Remove a collision pair.
        """
    def removeGeometryObject(self, name: str) -> None: 
        """
        removeGeometryObject( (GeometryModel)self, (str)name) -> None :
            Remove a GeometryObject. Remove also the collision pairs that contain the object.
        """
    @typing.overload
    def saveToBinary(self, buffer: StaticBuffer) -> None: 
        """
        saveToBinary( (GeometryModel)self, (str)filename) -> None :
            Saves *this inside a binary file.
        """
    @typing.overload
    def saveToBinary(self, buffer: StreamBuffer) -> None: ...
    @typing.overload
    def saveToBinary(self, filename: str) -> None: ...
    def saveToString(self) -> str: 
        """
        saveToString( (GeometryModel)self) -> str :
            Parses the current object to a string.
        """
    def saveToText(self, filename: str) -> None: 
        """
        saveToText( (GeometryModel)self, (str)filename) -> None :
            Saves *this inside a text file.
        """
    @staticmethod
    def saveToXML(arg1: GeometryModel, filename: str, tag_name: str) -> None: 
        """
        saveToXML( (GeometryModel)arg1, (str)filename, (str)tag_name) -> None :
            Saves *this inside a XML file.
        """
    def setCollisionPairs(self, collision_map_: numpy.ndarray, upper: bool = True) -> None: 
        """
        setCollisionPairs( (GeometryModel)self, (numpy.ndarray)collision_map [, (bool)upper=True]) -> None :
            Set the collision pairs from a given input array.
            Each entry of the input matrix defines the activation of a given collision pair(map[i,j] == True means that the pair (i,j) is active).
        """
    @property
    def collisionPairMapping(self) -> numpy.ndarray:
        """
        Matrix relating the collision pair ID to a pair of two GeometryObject indexes.

        :type: numpy.ndarray
        """
    @property
    def collisionPairs(self) -> Sequence[CollisionPair]:
        """
        Vector of collision pairs.

        :type: StdVec_CollisionPair
        """
    @property
    def geometryObjects(self) -> Sequence[GeometryObject]:
        """
        Vector of geometries objects.

        :type: StdVec_GeometryObject
        """
    @property
    def ngeoms(self) -> int:
        """
        Number of geometries contained in the Geometry Model.

        :type: int
        """
    __getstate_manages_dict__ = True
    __safe_for_unpickling__ = True
    pass

class GeometryNoMaterial(Boost.Python.instance):
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: GeometryNoMaterial) -> None: ...
    __instance_size__ = 40
    pass

class GeometryObject(Boost.Python.instance):
    """
    A wrapper on a collision geometry including its parent joint, parent frame, placement in parent joint's frame.
    """
    def __address__(self) -> int: 
        """
        __address__( (GeometryObject)self) -> int :
            Returns the address of the underlying C++ object.
        """
    def __copy__(self) -> GeometryObject: 
        """
        __copy__( (GeometryObject)self) -> GeometryObject :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> GeometryObject: 
        """
        __deepcopy__( (GeometryObject)self, (dict)memo) -> GeometryObject :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: GeometryObject, arg2: GeometryObject) -> object: 
        """
        __eq__( (GeometryObject)arg1, (GeometryObject)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(arg1: GeometryObject) -> tuple: 
        """
        __getinitargs__( (GeometryObject)arg1) -> tuple
        """
    @staticmethod
    def __getstate__(arg1: GeometryObject) -> tuple: 
        """
        __getstate__( (GeometryObject)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, self: str, name: int, parent_frame: int, parent_joint: object, collision_geometry_: SE3, placement_: str, mesh_path_: numpy.ndarray, mesh_scale_: bool, override_material_: numpy.ndarray, mesh_color_: str, mesh_texture_pathmesh_material: object) -> None: 
        """
        __init__( (object)self, (str)name, (int)parent_joint, (int)parent_frame, (SE3)placement, (object)collision_geometry [, (str)mesh_path [, (numpy.ndarray)mesh_scale [, (bool)override_material [, (numpy.ndarray)mesh_color [, (str)mesh_texture_path [, (object)mesh_material]]]]]]) -> None :
            Full constructor of a GeometryObject.
        """
    @typing.overload
    def __init__(self, name: str, parent_joint: int, collision_geometry: object, placement_: SE3, mesh_path_: str, mesh_scale_: numpy.ndarray, override_material_: bool, mesh_color_: numpy.ndarray, mesh_texture_path_: str, mesh_material: object) -> None: ...
    @typing.overload
    def __init__(self, name: str, parent_joint: int, parent_frame: int, placement: SE3, collision_geometry_: object, mesh_path_: str, mesh_scale_: numpy.ndarray, override_material_: bool, mesh_color_: numpy.ndarray, mesh_texture_path_: str, mesh_material: object) -> None: ...
    @typing.overload
    def __init__(self, name: str, parent_joint: int, placement: SE3, collision_geometry_: object, mesh_path_: str, mesh_scale_: numpy.ndarray, override_material_: bool, mesh_color_: numpy.ndarray, mesh_texture_path_: str, mesh_material: object) -> None: ...
    @typing.overload
    def __init__(self, otherGeometryObject: GeometryObject) -> None: ...
    @staticmethod
    def __ne__(arg1: GeometryObject, arg2: GeometryObject) -> object: 
        """
        __ne__( (GeometryObject)arg1, (GeometryObject)arg2) -> object
        """
    @staticmethod
    def __setstate__(arg1: GeometryObject, arg2: tuple) -> None: 
        """
        __setstate__( (GeometryObject)arg1, (tuple)arg2) -> None
        """
    def clone(self) -> GeometryObject: 
        """
        clone( (GeometryObject)self) -> GeometryObject :
            Perform a deep copy of this. It will create a copy of the underlying FCL geometry.
        """
    def copy(self) -> GeometryObject: 
        """
        copy( (GeometryObject)self) -> GeometryObject :
            Returns a copy of *this.
        """
    @typing.overload
    def loadFromBinary(self, buffer: StaticBuffer) -> None: 
        """
        loadFromBinary( (GeometryObject)self, (str)filename) -> None :
            Loads *this from a binary file.
        """
    @typing.overload
    def loadFromBinary(self, buffer: StreamBuffer) -> None: ...
    @typing.overload
    def loadFromBinary(self, filename: str) -> None: ...
    def loadFromString(self, string: str) -> None: 
        """
        loadFromString( (GeometryObject)self, (str)string) -> None :
            Parses from the input string the content of the current object.
        """
    def loadFromText(self, filename: str) -> None: 
        """
        loadFromText( (GeometryObject)self, (str)filename) -> None :
            Loads *this from a text file.
        """
    def loadFromXML(self, filename: str, tag_name: str) -> None: 
        """
        loadFromXML( (GeometryObject)self, (str)filename, (str)tag_name) -> None :
            Loads *this from a XML file.
        """
    @typing.overload
    def saveToBinary(self, buffer: StaticBuffer) -> None: 
        """
        saveToBinary( (GeometryObject)self, (str)filename) -> None :
            Saves *this inside a binary file.
        """
    @typing.overload
    def saveToBinary(self, buffer: StreamBuffer) -> None: ...
    @typing.overload
    def saveToBinary(self, filename: str) -> None: ...
    def saveToString(self) -> str: 
        """
        saveToString( (GeometryObject)self) -> str :
            Parses the current object to a string.
        """
    def saveToText(self, filename: str) -> None: 
        """
        saveToText( (GeometryObject)self, (str)filename) -> None :
            Saves *this inside a text file.
        """
    @staticmethod
    def saveToXML(arg1: GeometryObject, filename: str, tag_name: str) -> None: 
        """
        saveToXML( (GeometryObject)arg1, (str)filename, (str)tag_name) -> None :
            Saves *this inside a XML file.
        """
    @property
    def disableCollision(self) -> bool:
        """
        If true, no collision or distance check will be done between the Geometry and any other geometry.

        :type: bool
        """
    @property
    def geometry(self) -> typing.Optional[coal.CollisionGeometry]:
        """
        The FCL CollisionGeometry associated to the given GeometryObject.

        :type: typing.Optional[coal.CollisionGeometry]
        """
    @property
    def meshColor(self) -> numpy.ndarray:
        """
        Color rgba of the mesh.

        :type: numpy.ndarray
        """
    @property
    def meshMaterial(self) -> typing.Union[GeometryNoMaterial, GeometryPhongMaterial]:
        """
        Material associated to the mesh (applied only if overrideMaterial is True)

        :type: typing.Union[GeometryNoMaterial, GeometryPhongMaterial]
        """
    @property
    def meshPath(self) -> str:
        """
        Path to the mesh file.

        :type: str
        """
    @property
    def meshScale(self) -> numpy.ndarray:
        """
        Scaling parameter of the mesh.

        :type: numpy.ndarray
        """
    @property
    def meshTexturePath(self) -> str:
        """
        Path to the mesh texture file.

        :type: str
        """
    @property
    def name(self) -> str:
        """
        Name associated to the given GeometryObject.

        :type: str
        """
    @property
    def overrideMaterial(self) -> bool:
        """
        Boolean that tells whether material information is stored inside the given GeometryObject.

        :type: bool
        """
    @property
    def parentFrame(self) -> int:
        """
        Index of the parent frame.

        :type: int
        """
    @property
    def parentJoint(self) -> int:
        """
        Index of the parent joint.

        :type: int
        """
    @property
    def placement(self) -> SE3:
        """
        Position of geometry object in parent joint's frame.

        :type: SE3
        """
    __getstate_manages_dict__ = True
    __safe_for_unpickling__ = True
    pass

class GeometryPhongMaterial(Boost.Python.instance):
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: GeometryPhongMaterial) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: numpy.ndarray, arg3: numpy.ndarray, arg4: float) -> None: ...
    @property
    def meshEmissionColor(self) -> numpy.ndarray:
        """
        RGBA emission (ambient) color value of the mesh

        :type: numpy.ndarray
        """
    @property
    def meshShininess(self) -> float:
        """
        Shininess associated to the specular lighting model (between 0 and 1)

        :type: float
        """
    @property
    def meshSpecularColor(self) -> numpy.ndarray:
        """
        RGBA specular value of the mesh

        :type: numpy.ndarray
        """
    __instance_size__ = 112
    pass

class Inertia(Boost.Python.instance):
    """
    This class represenses a sparse version of a Spatial Inertia and its is defined by its mass, its center of mass location and the rotational inertia expressed around this center of mass.

    Supported operations ...
    """
    @staticmethod
    def FromBox(mass: float, length_x: float, length_y: float, length_z: float) -> Inertia: 
        """
        FromBox( (float)mass, (float)length_x, (float)length_y, (float)length_z) -> Inertia :
            Returns the Inertia of a box shape with a mass and of dimension the semi axis of length_{x,y,z}.
        """
    @staticmethod
    def FromCapsule(mass: float, radius: float, height: float) -> Inertia: 
        """
        FromCapsule( (float)mass, (float)radius, (float)height) -> Inertia :
            Computes the Inertia of a capsule defined by its mass, radius and length along the Z axis. Assumes a uniform density.
        """
    @staticmethod
    def FromCylinder(mass: float, radius: float, length: float) -> Inertia: 
        """
        FromCylinder( (float)mass, (float)radius, (float)length) -> Inertia :
            Returns the Inertia of a cylinder defined by its mass, radius and length along the Z axis.
        """
    @staticmethod
    def FromDynamicParameters(dynamic_parameters: numpy.ndarray) -> Inertia: 
        """
        FromDynamicParameters( (numpy.ndarray)dynamic_parameters) -> Inertia :
            Builds and inertia matrix from a vector of dynamic parameters.
            The parameters are given as dynamic_parameters = [m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T where I = I_C + mS^T(c)S(c) and I_C has its origin at the barycenter.
        """
    @staticmethod
    def FromEllipsoid(mass: float, length_x: float, length_y: float, length_z: float) -> Inertia: 
        """
        FromEllipsoid( (float)mass, (float)length_x, (float)length_y, (float)length_z) -> Inertia :
            Returns the Inertia of an ellipsoid shape defined by a mass and given dimensions the semi-axis of values length_{x,y,z}.
        """
    @staticmethod
    def FromLogCholeskyParameters(log_cholesky_parameters: LogCholeskyParameters) -> Inertia: 
        """
        FromLogCholeskyParameters( (LogCholeskyParameters)log_cholesky_parameters) -> Inertia :
            Returns the Inertia created from log Cholesky parameters.
        """
    @staticmethod
    def FromPseudoInertia(pseudo_inertia: PseudoInertia) -> Inertia: 
        """
        FromPseudoInertia( (PseudoInertia)pseudo_inertia) -> Inertia :
            Returns the Inertia created from a pseudo inertia object.
        """
    @staticmethod
    def FromSphere(mass: float, radius: float) -> Inertia: 
        """
        FromSphere( (float)mass, (float)radius) -> Inertia :
            Returns the Inertia of a sphere defined by a given mass and radius.
        """
    @staticmethod
    def Identity() -> Inertia: 
        """
        Identity() -> Inertia :
            Returns the identity Inertia.
        """
    @staticmethod
    def Random() -> Inertia: 
        """
        Random() -> Inertia :
            Returns a random Inertia.
        """
    @staticmethod
    def Zero() -> Inertia: 
        """
        Zero() -> Inertia :
            Returns the zero Inertia.
        """
    @staticmethod
    def __add__(arg1: Inertia, arg2: Inertia) -> object: 
        """
        __add__( (Inertia)arg1, (Inertia)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __array__(arg1: Inertia) -> numpy.ndarray: 
        """
        __array__( (Inertia)arg1) -> numpy.ndarray
        """
    @typing.overload
    def __array__(self, dtype: object = None, copy: object = None) -> numpy.ndarray: ...
    def __copy__(self) -> Inertia: 
        """
        __copy__( (Inertia)self) -> Inertia :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> Inertia: 
        """
        __deepcopy__( (Inertia)self, (dict)memo) -> Inertia :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: Inertia, arg2: Inertia) -> object: 
        """
        __eq__( (Inertia)arg1, (Inertia)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(arg1: Inertia) -> tuple: 
        """
        __getinitargs__( (Inertia)arg1) -> tuple
        """
    @staticmethod
    def __iadd__(arg1: object, arg2: Inertia) -> object: 
        """
        __iadd__( (object)arg1, (Inertia)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, clone: Inertia) -> object: 
        """
        __init__( (object)arg1, (float)mass, (numpy.ndarray)lever, (numpy.ndarray)inertia) -> object :
            Initialize from mass, lever and 3d inertia.
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, mass: float, lever: numpy.ndarray, inertia: numpy.ndarray) -> object: ...
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, clone: Inertia) -> None: ...
    @staticmethod
    def __isub__(arg1: object, arg2: Inertia) -> object: 
        """
        __isub__( (object)arg1, (Inertia)arg2) -> object
        """
    @staticmethod
    def __mul__(arg1: Inertia, arg2: Motion) -> object: 
        """
        __mul__( (Inertia)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __ne__(arg1: Inertia, arg2: Inertia) -> object: 
        """
        __ne__( (Inertia)arg1, (Inertia)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: Inertia) -> object: 
        """
        __repr__( (Inertia)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: Inertia) -> object: 
        """
        __str__( (Inertia)arg1) -> object
        """
    @staticmethod
    def __sub__(arg1: Inertia, arg2: Inertia) -> object: 
        """
        __sub__( (Inertia)arg1, (Inertia)arg2) -> object
        """
    @staticmethod
    def cast(arg1: Inertia) -> Inertia: 
        """
        cast( (Inertia)arg1) -> Inertia :
            Returns a cast of *this.
        """
    def copy(self) -> Inertia: 
        """
        copy( (Inertia)self) -> Inertia :
            Returns a copy of *this.
        """
    def inverse(self) -> numpy.ndarray: 
        """
        inverse( (Inertia)self) -> numpy.ndarray
        """
    def isApprox(self, other_: Inertia, prec: float = 1e-12) -> bool: 
        """
        isApprox( (Inertia)self, (Inertia)other [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to other, within the precision given by prec.
        """
    def isZero(self, prec: float = 1e-12) -> bool: 
        """
        isZero( (Inertia)self [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to the zero Inertia, within the precision given by prec.
        """
    def ivx(self, v: object) -> numpy.ndarray: 
        """
        ivx( (Inertia)self, (object)v) -> numpy.ndarray :
            Returns the result of I vx, a 6x6 matrix.
        """
    def matrix(self) -> numpy.ndarray: 
        """
        matrix( (Inertia)self) -> numpy.ndarray
        """
    def se3Action(self, M: SE3) -> Inertia: 
        """
        se3Action( (Inertia)self, (SE3)M) -> Inertia :
            Returns the result of the action of M on *this.
        """
    def se3ActionInverse(self, M: SE3) -> Inertia: 
        """
        se3ActionInverse( (Inertia)self, (SE3)M) -> Inertia :
            Returns the result of the action of the inverse of M on *this.
        """
    def setIdentity(self) -> None: 
        """
        setIdentity( (Inertia)self) -> None :
            Set *this to be the Identity inertia.
        """
    def setRandom(self) -> None: 
        """
        setRandom( (Inertia)self) -> None :
            Set all the components of *this to random values.
        """
    def setZero(self) -> None: 
        """
        setZero( (Inertia)self) -> None :
            Set all the components of *this to zero.
        """
    def toDynamicParameters(self) -> numpy.ndarray: 
        """
        toDynamicParameters( (Inertia)self) -> numpy.ndarray :
            Returns the representation of the matrix as a vector of dynamic parameters.
            The parameters are given as v = [m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T where I = I_C + mS^T(c)S(c) and I_C has its origin at the barycenter
        """
    def toPseudoInertia(self) -> PseudoInertia: 
        """
        toPseudoInertia( (Inertia)self) -> PseudoInertia :
            Returns the pseudo inertia representation of the inertia.
        """
    def variation(self, v: object) -> numpy.ndarray: 
        """
        variation( (Inertia)self, (object)v) -> numpy.ndarray :
            Returns the time derivative of the inertia.
        """
    def vtiv(self, v: object) -> float: 
        """
        vtiv( (Inertia)self, (object)v) -> float :
            Returns the result of v.T * Iv.
        """
    def vxi(self, v: object) -> numpy.ndarray: 
        """
        vxi( (Inertia)self, (object)v) -> numpy.ndarray :
            Returns the result of v x* I, a 6x6 matrix.
        """
    def vxiv(self, v: object) -> Force: 
        """
        vxiv( (Inertia)self, (object)v) -> Force :
            Returns the result of v x Iv.
        """
    @property
    def inertia(self) -> numpy.ndarray:
        """
        Rotational part of the Spatial Inertia, i.e. a symmetric matrix representing the rotational inertia around the center of mass.

        :type: numpy.ndarray
        """
    @property
    def lever(self) -> numpy.ndarray:
        """
        Center of mass location of the Spatial Inertia. It corresponds to the location of the center of mass regarding to the frame where the Spatial Inertia is expressed.

        :type: numpy.ndarray
        """
    @property
    def mass(self) -> float:
        """
        Mass of the Spatial Inertia.

        :type: float
        """
    @property
    def np(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    __safe_for_unpickling__ = True
    pass

class JointData(Boost.Python.instance):
    """
    Generic Joint Data
    """
    @staticmethod
    def __eq__(arg1: JointData, arg2: JointData) -> object: 
        """
        __eq__( (JointData)arg1, (JointData)arg2) -> object
        """
    def __init__(self, joint_data: object) -> None: 
        """
        __init__( (object)self, (object)joint_data) -> None
        """
    @staticmethod
    def __ne__(arg1: JointData, arg2: JointData) -> object: 
        """
        __ne__( (JointData)arg1, (JointData)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointData) -> object: 
        """
        __repr__( (JointData)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointData) -> object: 
        """
        __str__( (JointData)arg1) -> object
        """
    def extract(self) -> object: 
        """
        extract( (JointData)self) -> object :
            Returns a reference of the internal joint managed by the JointData
        """
    def shortname(self) -> str: 
        """
        shortname( (JointData)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    pass

class JointDataComposite(Boost.Python.instance):
    """
    JointDataComposite
    """
    @staticmethod
    def __eq__(arg1: JointDataComposite, arg2: JointDataComposite) -> object: 
        """
        __eq__( (JointDataComposite)arg1, (JointDataComposite)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, joint_data_vectors: StdVec_JointDataVector, nq: int, nv: int) -> None: ...
    @staticmethod
    def __ne__(arg1: JointDataComposite, arg2: JointDataComposite) -> object: 
        """
        __ne__( (JointDataComposite)arg1, (JointDataComposite)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataComposite) -> object: 
        """
        __repr__( (JointDataComposite)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataComposite) -> object: 
        """
        __str__( (JointDataComposite)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataComposite)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def StU(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def iMlast(self) -> Sequence[SE3]:
        """
        :type: StdVec_SE3
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joints(self) -> Sequence[JointData]:
        """
        :type: StdVec_JointDataVector
        """
    @property
    def pjMi(self) -> Sequence[SE3]:
        """
        :type: StdVec_SE3
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 432
    pass

class JointDataFreeFlyer(Boost.Python.instance):
    """
    JointDataFreeFlyer
    """
    @staticmethod
    def __eq__(arg1: JointDataFreeFlyer, arg2: JointDataFreeFlyer) -> object: 
        """
        __eq__( (JointDataFreeFlyer)arg1, (JointDataFreeFlyer)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataFreeFlyer, arg2: JointDataFreeFlyer) -> object: 
        """
        __ne__( (JointDataFreeFlyer)arg1, (JointDataFreeFlyer)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataFreeFlyer) -> object: 
        """
        __repr__( (JointDataFreeFlyer)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataFreeFlyer) -> object: 
        """
        __str__( (JointDataFreeFlyer)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataFreeFlyer)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 1472
    pass

class JointDataHX(Boost.Python.instance):
    """
    JointDataHX
    """
    @staticmethod
    def __eq__(arg1: JointDataHX, arg2: JointDataHX) -> object: 
        """
        __eq__( (JointDataHX)arg1, (JointDataHX)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataHX, arg2: JointDataHX) -> object: 
        """
        __ne__( (JointDataHX)arg1, (JointDataHX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataHX) -> object: 
        """
        __repr__( (JointDataHX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataHX) -> object: 
        """
        __str__( (JointDataHX)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataHX)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 240
    pass

class JointDataHY(Boost.Python.instance):
    """
    JointDataHY
    """
    @staticmethod
    def __eq__(arg1: JointDataHY, arg2: JointDataHY) -> object: 
        """
        __eq__( (JointDataHY)arg1, (JointDataHY)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataHY, arg2: JointDataHY) -> object: 
        """
        __ne__( (JointDataHY)arg1, (JointDataHY)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataHY) -> object: 
        """
        __repr__( (JointDataHY)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataHY) -> object: 
        """
        __str__( (JointDataHY)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataHY)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 240
    pass

class JointDataHZ(Boost.Python.instance):
    """
    JointDataHZ
    """
    @staticmethod
    def __eq__(arg1: JointDataHZ, arg2: JointDataHZ) -> object: 
        """
        __eq__( (JointDataHZ)arg1, (JointDataHZ)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataHZ, arg2: JointDataHZ) -> object: 
        """
        __ne__( (JointDataHZ)arg1, (JointDataHZ)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataHZ) -> object: 
        """
        __repr__( (JointDataHZ)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataHZ) -> object: 
        """
        __str__( (JointDataHZ)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataHZ)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 240
    pass

class JointDataHelicalUnaligned(Boost.Python.instance):
    """
    JointDataHelicalUnaligned
    """
    @staticmethod
    def __eq__(arg1: JointDataHelicalUnaligned, arg2: JointDataHelicalUnaligned) -> object: 
        """
        __eq__( (JointDataHelicalUnaligned)arg1, (JointDataHelicalUnaligned)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, axis: numpy.ndarray) -> None: ...
    @staticmethod
    def __ne__(arg1: JointDataHelicalUnaligned, arg2: JointDataHelicalUnaligned) -> object: 
        """
        __ne__( (JointDataHelicalUnaligned)arg1, (JointDataHelicalUnaligned)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataHelicalUnaligned) -> object: 
        """
        __repr__( (JointDataHelicalUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataHelicalUnaligned) -> object: 
        """
        __str__( (JointDataHelicalUnaligned)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataHelicalUnaligned)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 352
    pass

class JointDataMimic_JointDataRX(Boost.Python.instance):
    """
    JointDataMimic_JointDataRX
    """
    @staticmethod
    def __eq__(arg1: JointDataMimic_JointDataRX, arg2: JointDataMimic_JointDataRX) -> object: 
        """
        __eq__( (JointDataMimic_JointDataRX)arg1, (JointDataMimic_JointDataRX)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataMimic_JointDataRX, arg2: JointDataMimic_JointDataRX) -> object: 
        """
        __ne__( (JointDataMimic_JointDataRX)arg1, (JointDataMimic_JointDataRX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataMimic_JointDataRX) -> object: 
        """
        __repr__( (JointDataMimic_JointDataRX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataMimic_JointDataRX) -> object: 
        """
        __str__( (JointDataMimic_JointDataRX)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataMimic_JointDataRX)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 272
    pass

class JointDataMimic_JointDataRY(Boost.Python.instance):
    """
    JointDataMimic_JointDataRY
    """
    @staticmethod
    def __eq__(arg1: JointDataMimic_JointDataRY, arg2: JointDataMimic_JointDataRY) -> object: 
        """
        __eq__( (JointDataMimic_JointDataRY)arg1, (JointDataMimic_JointDataRY)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataMimic_JointDataRY, arg2: JointDataMimic_JointDataRY) -> object: 
        """
        __ne__( (JointDataMimic_JointDataRY)arg1, (JointDataMimic_JointDataRY)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataMimic_JointDataRY) -> object: 
        """
        __repr__( (JointDataMimic_JointDataRY)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataMimic_JointDataRY) -> object: 
        """
        __str__( (JointDataMimic_JointDataRY)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataMimic_JointDataRY)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 272
    pass

class JointDataMimic_JointDataRZ(Boost.Python.instance):
    """
    JointDataMimic_JointDataRZ
    """
    @staticmethod
    def __eq__(arg1: JointDataMimic_JointDataRZ, arg2: JointDataMimic_JointDataRZ) -> object: 
        """
        __eq__( (JointDataMimic_JointDataRZ)arg1, (JointDataMimic_JointDataRZ)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataMimic_JointDataRZ, arg2: JointDataMimic_JointDataRZ) -> object: 
        """
        __ne__( (JointDataMimic_JointDataRZ)arg1, (JointDataMimic_JointDataRZ)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataMimic_JointDataRZ) -> object: 
        """
        __repr__( (JointDataMimic_JointDataRZ)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataMimic_JointDataRZ) -> object: 
        """
        __str__( (JointDataMimic_JointDataRZ)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataMimic_JointDataRZ)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 272
    pass

class JointDataPX(Boost.Python.instance):
    """
    JointDataPX
    """
    @staticmethod
    def __eq__(arg1: JointDataPX, arg2: JointDataPX) -> object: 
        """
        __eq__( (JointDataPX)arg1, (JointDataPX)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataPX, arg2: JointDataPX) -> object: 
        """
        __ne__( (JointDataPX)arg1, (JointDataPX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataPX) -> object: 
        """
        __repr__( (JointDataPX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataPX) -> object: 
        """
        __str__( (JointDataPX)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataPX)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 208
    pass

class JointDataPY(Boost.Python.instance):
    """
    JointDataPY
    """
    @staticmethod
    def __eq__(arg1: JointDataPY, arg2: JointDataPY) -> object: 
        """
        __eq__( (JointDataPY)arg1, (JointDataPY)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataPY, arg2: JointDataPY) -> object: 
        """
        __ne__( (JointDataPY)arg1, (JointDataPY)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataPY) -> object: 
        """
        __repr__( (JointDataPY)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataPY) -> object: 
        """
        __str__( (JointDataPY)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataPY)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 208
    pass

class JointDataPZ(Boost.Python.instance):
    """
    JointDataPZ
    """
    @staticmethod
    def __eq__(arg1: JointDataPZ, arg2: JointDataPZ) -> object: 
        """
        __eq__( (JointDataPZ)arg1, (JointDataPZ)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataPZ, arg2: JointDataPZ) -> object: 
        """
        __ne__( (JointDataPZ)arg1, (JointDataPZ)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataPZ) -> object: 
        """
        __repr__( (JointDataPZ)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataPZ) -> object: 
        """
        __str__( (JointDataPZ)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataPZ)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 208
    pass

class JointDataPlanar(Boost.Python.instance):
    """
    JointDataPlanar
    """
    @staticmethod
    def __eq__(arg1: JointDataPlanar, arg2: JointDataPlanar) -> object: 
        """
        __eq__( (JointDataPlanar)arg1, (JointDataPlanar)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataPlanar, arg2: JointDataPlanar) -> object: 
        """
        __ne__( (JointDataPlanar)arg1, (JointDataPlanar)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataPlanar) -> object: 
        """
        __repr__( (JointDataPlanar)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataPlanar) -> object: 
        """
        __str__( (JointDataPlanar)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataPlanar)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def StU(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 672
    pass

class JointDataPrismaticUnaligned(Boost.Python.instance):
    """
    JointDataPrismaticUnaligned
    """
    @staticmethod
    def __eq__(arg1: JointDataPrismaticUnaligned, arg2: JointDataPrismaticUnaligned) -> object: 
        """
        __eq__( (JointDataPrismaticUnaligned)arg1, (JointDataPrismaticUnaligned)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, axis: numpy.ndarray) -> None: ...
    @staticmethod
    def __ne__(arg1: JointDataPrismaticUnaligned, arg2: JointDataPrismaticUnaligned) -> object: 
        """
        __ne__( (JointDataPrismaticUnaligned)arg1, (JointDataPrismaticUnaligned)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataPrismaticUnaligned) -> object: 
        """
        __repr__( (JointDataPrismaticUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataPrismaticUnaligned) -> object: 
        """
        __str__( (JointDataPrismaticUnaligned)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataPrismaticUnaligned)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 272
    pass

class JointDataRUBX(Boost.Python.instance):
    """
    JointDataRUBX
    """
    @staticmethod
    def __eq__(arg1: JointDataRUBX, arg2: JointDataRUBX) -> object: 
        """
        __eq__( (JointDataRUBX)arg1, (JointDataRUBX)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataRUBX, arg2: JointDataRUBX) -> object: 
        """
        __ne__( (JointDataRUBX)arg1, (JointDataRUBX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataRUBX) -> object: 
        """
        __repr__( (JointDataRUBX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataRUBX) -> object: 
        """
        __str__( (JointDataRUBX)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataRUBX)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 224
    pass

class JointDataRUBY(Boost.Python.instance):
    """
    JointDataRUBY
    """
    @staticmethod
    def __eq__(arg1: JointDataRUBY, arg2: JointDataRUBY) -> object: 
        """
        __eq__( (JointDataRUBY)arg1, (JointDataRUBY)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataRUBY, arg2: JointDataRUBY) -> object: 
        """
        __ne__( (JointDataRUBY)arg1, (JointDataRUBY)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataRUBY) -> object: 
        """
        __repr__( (JointDataRUBY)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataRUBY) -> object: 
        """
        __str__( (JointDataRUBY)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataRUBY)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 224
    pass

class JointDataRUBZ(Boost.Python.instance):
    """
    JointDataRUBZ
    """
    @staticmethod
    def __eq__(arg1: JointDataRUBZ, arg2: JointDataRUBZ) -> object: 
        """
        __eq__( (JointDataRUBZ)arg1, (JointDataRUBZ)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataRUBZ, arg2: JointDataRUBZ) -> object: 
        """
        __ne__( (JointDataRUBZ)arg1, (JointDataRUBZ)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataRUBZ) -> object: 
        """
        __repr__( (JointDataRUBZ)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataRUBZ) -> object: 
        """
        __str__( (JointDataRUBZ)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataRUBZ)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 224
    pass

class JointDataRX(Boost.Python.instance):
    """
    JointDataRX
    """
    @staticmethod
    def __eq__(arg1: JointDataRX, arg2: JointDataRX) -> object: 
        """
        __eq__( (JointDataRX)arg1, (JointDataRX)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataRX, arg2: JointDataRX) -> object: 
        """
        __ne__( (JointDataRX)arg1, (JointDataRX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataRX) -> object: 
        """
        __repr__( (JointDataRX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataRX) -> object: 
        """
        __str__( (JointDataRX)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataRX)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 224
    pass

class JointDataRY(Boost.Python.instance):
    """
    JointDataRY
    """
    @staticmethod
    def __eq__(arg1: JointDataRY, arg2: JointDataRY) -> object: 
        """
        __eq__( (JointDataRY)arg1, (JointDataRY)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataRY, arg2: JointDataRY) -> object: 
        """
        __ne__( (JointDataRY)arg1, (JointDataRY)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataRY) -> object: 
        """
        __repr__( (JointDataRY)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataRY) -> object: 
        """
        __str__( (JointDataRY)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataRY)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 224
    pass

class JointDataRZ(Boost.Python.instance):
    """
    JointDataRZ
    """
    @staticmethod
    def __eq__(arg1: JointDataRZ, arg2: JointDataRZ) -> object: 
        """
        __eq__( (JointDataRZ)arg1, (JointDataRZ)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataRZ, arg2: JointDataRZ) -> object: 
        """
        __ne__( (JointDataRZ)arg1, (JointDataRZ)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataRZ) -> object: 
        """
        __repr__( (JointDataRZ)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataRZ) -> object: 
        """
        __str__( (JointDataRZ)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataRZ)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 224
    pass

class JointDataRevoluteUnaligned(Boost.Python.instance):
    """
    JointDataRevoluteUnaligned
    """
    @staticmethod
    def __eq__(arg1: JointDataRevoluteUnaligned, arg2: JointDataRevoluteUnaligned) -> object: 
        """
        __eq__( (JointDataRevoluteUnaligned)arg1, (JointDataRevoluteUnaligned)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, axis: numpy.ndarray) -> None: ...
    @staticmethod
    def __ne__(arg1: JointDataRevoluteUnaligned, arg2: JointDataRevoluteUnaligned) -> object: 
        """
        __ne__( (JointDataRevoluteUnaligned)arg1, (JointDataRevoluteUnaligned)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataRevoluteUnaligned) -> object: 
        """
        __repr__( (JointDataRevoluteUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataRevoluteUnaligned) -> object: 
        """
        __str__( (JointDataRevoluteUnaligned)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataRevoluteUnaligned)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 336
    pass

class JointDataRevoluteUnboundedUnalignedTpl(Boost.Python.instance):
    """
    JointDataRevoluteUnboundedUnalignedTpl
    """
    @staticmethod
    def __eq__(arg1: JointDataRevoluteUnboundedUnalignedTpl, arg2: JointDataRevoluteUnboundedUnalignedTpl) -> object: 
        """
        __eq__( (JointDataRevoluteUnboundedUnalignedTpl)arg1, (JointDataRevoluteUnboundedUnalignedTpl)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataRevoluteUnboundedUnalignedTpl, arg2: JointDataRevoluteUnboundedUnalignedTpl) -> object: 
        """
        __ne__( (JointDataRevoluteUnboundedUnalignedTpl)arg1, (JointDataRevoluteUnboundedUnalignedTpl)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataRevoluteUnboundedUnalignedTpl) -> object: 
        """
        __repr__( (JointDataRevoluteUnboundedUnalignedTpl)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataRevoluteUnboundedUnalignedTpl) -> object: 
        """
        __str__( (JointDataRevoluteUnboundedUnalignedTpl)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataRevoluteUnboundedUnalignedTpl)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 352
    pass

class JointDataSpherical(Boost.Python.instance):
    """
    JointDataSpherical
    """
    @staticmethod
    def __eq__(arg1: JointDataSpherical, arg2: JointDataSpherical) -> object: 
        """
        __eq__( (JointDataSpherical)arg1, (JointDataSpherical)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataSpherical, arg2: JointDataSpherical) -> object: 
        """
        __ne__( (JointDataSpherical)arg1, (JointDataSpherical)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataSpherical) -> object: 
        """
        __repr__( (JointDataSpherical)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataSpherical) -> object: 
        """
        __str__( (JointDataSpherical)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataSpherical)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 672
    pass

class JointDataSphericalZYX(Boost.Python.instance):
    """
    JointDataSphericalZYX
    """
    @staticmethod
    def __eq__(arg1: JointDataSphericalZYX, arg2: JointDataSphericalZYX) -> object: 
        """
        __eq__( (JointDataSphericalZYX)arg1, (JointDataSphericalZYX)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataSphericalZYX, arg2: JointDataSphericalZYX) -> object: 
        """
        __ne__( (JointDataSphericalZYX)arg1, (JointDataSphericalZYX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataSphericalZYX) -> object: 
        """
        __repr__( (JointDataSphericalZYX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataSphericalZYX) -> object: 
        """
        __str__( (JointDataSphericalZYX)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataSphericalZYX)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def StU(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 752
    pass

class JointDataTranslation(Boost.Python.instance):
    """
    JointDataTranslation
    """
    @staticmethod
    def __eq__(arg1: JointDataTranslation, arg2: JointDataTranslation) -> object: 
        """
        __eq__( (JointDataTranslation)arg1, (JointDataTranslation)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataTranslation, arg2: JointDataTranslation) -> object: 
        """
        __ne__( (JointDataTranslation)arg1, (JointDataTranslation)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataTranslation) -> object: 
        """
        __repr__( (JointDataTranslation)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataTranslation) -> object: 
        """
        __str__( (JointDataTranslation)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataTranslation)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 592
    pass

class JointDataUniversal(Boost.Python.instance):
    """
    JointDataUniversal
    """
    @staticmethod
    def __eq__(arg1: JointDataUniversal, arg2: JointDataUniversal) -> object: 
        """
        __eq__( (JointDataUniversal)arg1, (JointDataUniversal)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(arg1: JointDataUniversal, arg2: JointDataUniversal) -> object: 
        """
        __ne__( (JointDataUniversal)arg1, (JointDataUniversal)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointDataUniversal) -> object: 
        """
        __repr__( (JointDataUniversal)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointDataUniversal) -> object: 
        """
        __str__( (JointDataUniversal)arg1) -> object
        """
    def shortname(self) -> str: 
        """
        shortname( (JointDataUniversal)self) -> str
        """
    @property
    def Dinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def M(self) -> SE3:
        """
        :type: SE3
        """
    @property
    def S(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def U(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def UDinv(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def c(self) -> Motion:
        """
        :type: Motion
        """
    @property
    def joint_q(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def joint_v(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def v(self) -> Motion:
        """
        :type: Motion
        """
    __instance_size__ = 512
    pass

class JointModel(Boost.Python.instance):
    """
    Generic Joint Model
    """
    @staticmethod
    def __eq__(arg1: JointModel, arg2: JointModel) -> object: 
        """
        __eq__( (JointModel)arg1, (JointModel)arg2) -> object
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None :
            Default constructor
        """
    @typing.overload
    def __init__(self, other: JointModel) -> None: ...
    @staticmethod
    def __ne__(arg1: JointModel, arg2: JointModel) -> object: 
        """
        __ne__( (JointModel)arg1, (JointModel)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModel) -> object: 
        """
        __repr__( (JointModel)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModel) -> object: 
        """
        __str__( (JointModel)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointData, q: numpy.ndarray) -> None: 
        """
        calc( (JointModel)self, (JointData)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointData, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointData: 
        """
        createData( (JointModel)self) -> JointData :
            Create data associated to the joint model.
        """
    def extract(self) -> object: 
        """
        extract( (JointModel)self) -> object :
            Returns a reference of the internal joint managed by the JointModel
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModel)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModel)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModel)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelComposite(Boost.Python.instance):
    """
    JointModelComposite
    """
    @staticmethod
    def __eq__(arg1: JointModelComposite, arg2: JointModelComposite) -> object: 
        """
        __eq__( (JointModelComposite)arg1, (JointModelComposite)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, joint_model: JointModel) -> object: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, joint_model: JointModel, joint_placement: SE3) -> object: ...
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, size: int) -> None: ...
    @staticmethod
    def __ne__(arg1: JointModelComposite, arg2: JointModelComposite) -> object: 
        """
        __ne__( (JointModelComposite)arg1, (JointModelComposite)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelComposite) -> object: 
        """
        __repr__( (JointModelComposite)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelComposite) -> object: 
        """
        __str__( (JointModelComposite)arg1) -> object
        """
    def addJoint(self, joint_model_: JointModel, joint_placement: SE3 = SE3(array([[1., 0., 0., 0.],[0., 1., 0., 0.],[0., 0., 1., 0.],[0., 0., 0., 1.]]))) -> JointModelComposite: 
        """
        addJoint( (JointModelComposite)self, (JointModel)joint_model [, (SE3)joint_placement=SE3(array([[1., 0., 0., 0.],[0., 1., 0., 0.],[0., 0., 1., 0.],[0., 0., 0., 1.]]))]) -> JointModelComposite :
            Add a joint to the vector of joints.
        """
    @typing.overload
    def calc(self, jdata: JointDataComposite, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelComposite)self, (JointDataComposite)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataComposite, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataComposite: 
        """
        createData( (JointModelComposite)self) -> JointDataComposite :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelComposite)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelComposite)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelComposite)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def jointPlacements(self) -> Sequence[SE3]:
        """
        :type: StdVec_SE3
        """
    @property
    def joints(self) -> Sequence[JointModel]:
        """
        :type: StdVec_JointModelVector
        """
    @property
    def njoints(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelFreeFlyer(Boost.Python.instance):
    """
    JointModelFreeFlyer
    """
    @staticmethod
    def __eq__(arg1: JointModelFreeFlyer, arg2: JointModelFreeFlyer) -> object: 
        """
        __eq__( (JointModelFreeFlyer)arg1, (JointModelFreeFlyer)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelFreeFlyer, arg2: JointModelFreeFlyer) -> object: 
        """
        __ne__( (JointModelFreeFlyer)arg1, (JointModelFreeFlyer)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelFreeFlyer) -> object: 
        """
        __repr__( (JointModelFreeFlyer)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelFreeFlyer) -> object: 
        """
        __str__( (JointModelFreeFlyer)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataFreeFlyer, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelFreeFlyer)self, (JointDataFreeFlyer)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataFreeFlyer, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataFreeFlyer: 
        """
        createData( (JointModelFreeFlyer)self) -> JointDataFreeFlyer :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelFreeFlyer)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelFreeFlyer)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelFreeFlyer)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelHX(Boost.Python.instance):
    """
    JointModelHX
    """
    @staticmethod
    def __eq__(arg1: JointModelHX, arg2: JointModelHX) -> object: 
        """
        __eq__( (JointModelHX)arg1, (JointModelHX)arg2) -> object
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @typing.overload
    def __init__(self, pitch: float) -> None: ...
    @staticmethod
    def __ne__(arg1: JointModelHX, arg2: JointModelHX) -> object: 
        """
        __ne__( (JointModelHX)arg1, (JointModelHX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelHX) -> object: 
        """
        __repr__( (JointModelHX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelHX) -> object: 
        """
        __str__( (JointModelHX)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataHX, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelHX)self, (JointDataHX)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataHX, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataHX: 
        """
        createData( (JointModelHX)self) -> JointDataHX :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelHX) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelHX)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelHX.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelHX)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelHX)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelHX)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    @property
    def pitch(self) -> float:
        """
        Pitch h of the JointModelHX.

        :type: float
        """
    pass

class JointModelHY(Boost.Python.instance):
    """
    JointModelHY
    """
    @staticmethod
    def __eq__(arg1: JointModelHY, arg2: JointModelHY) -> object: 
        """
        __eq__( (JointModelHY)arg1, (JointModelHY)arg2) -> object
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @typing.overload
    def __init__(self, pitch: float) -> None: ...
    @staticmethod
    def __ne__(arg1: JointModelHY, arg2: JointModelHY) -> object: 
        """
        __ne__( (JointModelHY)arg1, (JointModelHY)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelHY) -> object: 
        """
        __repr__( (JointModelHY)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelHY) -> object: 
        """
        __str__( (JointModelHY)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataHY, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelHY)self, (JointDataHY)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataHY, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataHY: 
        """
        createData( (JointModelHY)self) -> JointDataHY :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelHY) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelHY)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelHY.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelHY)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelHY)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelHY)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    @property
    def pitch(self) -> float:
        """
        Pitch h of the JointModelHY.

        :type: float
        """
    pass

class JointModelHZ(Boost.Python.instance):
    """
    JointModelHZ
    """
    @staticmethod
    def __eq__(arg1: JointModelHZ, arg2: JointModelHZ) -> object: 
        """
        __eq__( (JointModelHZ)arg1, (JointModelHZ)arg2) -> object
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @typing.overload
    def __init__(self, pitch: float) -> None: ...
    @staticmethod
    def __ne__(arg1: JointModelHZ, arg2: JointModelHZ) -> object: 
        """
        __ne__( (JointModelHZ)arg1, (JointModelHZ)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelHZ) -> object: 
        """
        __repr__( (JointModelHZ)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelHZ) -> object: 
        """
        __str__( (JointModelHZ)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataHZ, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelHZ)self, (JointDataHZ)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataHZ, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataHZ: 
        """
        createData( (JointModelHZ)self) -> JointDataHZ :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelHZ) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelHZ)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelHZ.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelHZ)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelHZ)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelHZ)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    @property
    def pitch(self) -> float:
        """
        Pitch h of the JointModelHZ.

        :type: float
        """
    pass

class JointModelHelicalUnaligned(Boost.Python.instance):
    """
    JointModelHelicalUnaligned
    """
    @staticmethod
    def __eq__(arg1: JointModelHelicalUnaligned, arg2: JointModelHelicalUnaligned) -> object: 
        """
        __eq__( (JointModelHelicalUnaligned)arg1, (JointModelHelicalUnaligned)arg2) -> object
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @typing.overload
    def __init__(self, axis: numpy.ndarray, pitch: float) -> None: ...
    @typing.overload
    def __init__(self, x: float, y: float, z: float, pitch: float) -> None: ...
    @staticmethod
    def __ne__(arg1: JointModelHelicalUnaligned, arg2: JointModelHelicalUnaligned) -> object: 
        """
        __ne__( (JointModelHelicalUnaligned)arg1, (JointModelHelicalUnaligned)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelHelicalUnaligned) -> object: 
        """
        __repr__( (JointModelHelicalUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelHelicalUnaligned) -> object: 
        """
        __str__( (JointModelHelicalUnaligned)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataHelicalUnaligned, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelHelicalUnaligned)self, (JointDataHelicalUnaligned)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataHelicalUnaligned, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataHelicalUnaligned: 
        """
        createData( (JointModelHelicalUnaligned)self) -> JointDataHelicalUnaligned :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelHelicalUnaligned)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelHelicalUnaligned)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelHelicalUnaligned)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def axis(self) -> numpy.ndarray:
        """
        Translation axis of the JointModelHelicalUnaligned.

        :type: numpy.ndarray
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    @property
    def pitch(self) -> float:
        """
        Pitch h of the JointModelHelicalUnaligned.

        :type: float
        """
    pass

class JointModelMimic_JointModelRX(Boost.Python.instance):
    """
    JointModelMimic_JointModelRX
    """
    @staticmethod
    def __eq__(arg1: JointModelMimic_JointModelRX, arg2: JointModelMimic_JointModelRX) -> object: 
        """
        __eq__( (JointModelMimic_JointModelRX)arg1, (JointModelMimic_JointModelRX)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelMimic_JointModelRX, arg2: JointModelMimic_JointModelRX) -> object: 
        """
        __ne__( (JointModelMimic_JointModelRX)arg1, (JointModelMimic_JointModelRX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelMimic_JointModelRX) -> object: 
        """
        __repr__( (JointModelMimic_JointModelRX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelMimic_JointModelRX) -> object: 
        """
        __str__( (JointModelMimic_JointModelRX)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataMimic_JointDataRX, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelMimic_JointModelRX)self, (JointDataMimic_JointDataRX)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataMimic_JointDataRX, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataMimic_JointDataRX: 
        """
        createData( (JointModelMimic_JointModelRX)self) -> JointDataMimic_JointDataRX :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelMimic_JointModelRX)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelMimic_JointModelRX)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelMimic_JointModelRX)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelMimic_JointModelRY(Boost.Python.instance):
    """
    JointModelMimic_JointModelRY
    """
    @staticmethod
    def __eq__(arg1: JointModelMimic_JointModelRY, arg2: JointModelMimic_JointModelRY) -> object: 
        """
        __eq__( (JointModelMimic_JointModelRY)arg1, (JointModelMimic_JointModelRY)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelMimic_JointModelRY, arg2: JointModelMimic_JointModelRY) -> object: 
        """
        __ne__( (JointModelMimic_JointModelRY)arg1, (JointModelMimic_JointModelRY)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelMimic_JointModelRY) -> object: 
        """
        __repr__( (JointModelMimic_JointModelRY)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelMimic_JointModelRY) -> object: 
        """
        __str__( (JointModelMimic_JointModelRY)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataMimic_JointDataRY, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelMimic_JointModelRY)self, (JointDataMimic_JointDataRY)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataMimic_JointDataRY, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataMimic_JointDataRY: 
        """
        createData( (JointModelMimic_JointModelRY)self) -> JointDataMimic_JointDataRY :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelMimic_JointModelRY)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelMimic_JointModelRY)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelMimic_JointModelRY)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelMimic_JointModelRZ(Boost.Python.instance):
    """
    JointModelMimic_JointModelRZ
    """
    @staticmethod
    def __eq__(arg1: JointModelMimic_JointModelRZ, arg2: JointModelMimic_JointModelRZ) -> object: 
        """
        __eq__( (JointModelMimic_JointModelRZ)arg1, (JointModelMimic_JointModelRZ)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelMimic_JointModelRZ, arg2: JointModelMimic_JointModelRZ) -> object: 
        """
        __ne__( (JointModelMimic_JointModelRZ)arg1, (JointModelMimic_JointModelRZ)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelMimic_JointModelRZ) -> object: 
        """
        __repr__( (JointModelMimic_JointModelRZ)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelMimic_JointModelRZ) -> object: 
        """
        __str__( (JointModelMimic_JointModelRZ)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataMimic_JointDataRZ, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelMimic_JointModelRZ)self, (JointDataMimic_JointDataRZ)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataMimic_JointDataRZ, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataMimic_JointDataRZ: 
        """
        createData( (JointModelMimic_JointModelRZ)self) -> JointDataMimic_JointDataRZ :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelMimic_JointModelRZ)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelMimic_JointModelRZ)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelMimic_JointModelRZ)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelPX(Boost.Python.instance):
    """
    JointModelPX
    """
    @staticmethod
    def __eq__(arg1: JointModelPX, arg2: JointModelPX) -> object: 
        """
        __eq__( (JointModelPX)arg1, (JointModelPX)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelPX, arg2: JointModelPX) -> object: 
        """
        __ne__( (JointModelPX)arg1, (JointModelPX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelPX) -> object: 
        """
        __repr__( (JointModelPX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelPX) -> object: 
        """
        __str__( (JointModelPX)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataPX, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelPX)self, (JointDataPX)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataPX, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataPX: 
        """
        createData( (JointModelPX)self) -> JointDataPX :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelPX) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelPX)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelPX.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelPX)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelPX)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelPX)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelPY(Boost.Python.instance):
    """
    JointModelPY
    """
    @staticmethod
    def __eq__(arg1: JointModelPY, arg2: JointModelPY) -> object: 
        """
        __eq__( (JointModelPY)arg1, (JointModelPY)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelPY, arg2: JointModelPY) -> object: 
        """
        __ne__( (JointModelPY)arg1, (JointModelPY)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelPY) -> object: 
        """
        __repr__( (JointModelPY)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelPY) -> object: 
        """
        __str__( (JointModelPY)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataPY, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelPY)self, (JointDataPY)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataPY, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataPY: 
        """
        createData( (JointModelPY)self) -> JointDataPY :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelPY) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelPY)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelPY.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelPY)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelPY)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelPY)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelPZ(Boost.Python.instance):
    """
    JointModelPZ
    """
    @staticmethod
    def __eq__(arg1: JointModelPZ, arg2: JointModelPZ) -> object: 
        """
        __eq__( (JointModelPZ)arg1, (JointModelPZ)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelPZ, arg2: JointModelPZ) -> object: 
        """
        __ne__( (JointModelPZ)arg1, (JointModelPZ)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelPZ) -> object: 
        """
        __repr__( (JointModelPZ)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelPZ) -> object: 
        """
        __str__( (JointModelPZ)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataPZ, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelPZ)self, (JointDataPZ)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataPZ, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataPZ: 
        """
        createData( (JointModelPZ)self) -> JointDataPZ :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelPZ) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelPZ)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelPZ.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelPZ)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelPZ)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelPZ)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelPlanar(Boost.Python.instance):
    """
    JointModelPlanar
    """
    @staticmethod
    def __eq__(arg1: JointModelPlanar, arg2: JointModelPlanar) -> object: 
        """
        __eq__( (JointModelPlanar)arg1, (JointModelPlanar)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelPlanar, arg2: JointModelPlanar) -> object: 
        """
        __ne__( (JointModelPlanar)arg1, (JointModelPlanar)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelPlanar) -> object: 
        """
        __repr__( (JointModelPlanar)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelPlanar) -> object: 
        """
        __str__( (JointModelPlanar)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataPlanar, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelPlanar)self, (JointDataPlanar)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataPlanar, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataPlanar: 
        """
        createData( (JointModelPlanar)self) -> JointDataPlanar :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelPlanar)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelPlanar)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelPlanar)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelPrismaticUnaligned(Boost.Python.instance):
    """
    JointModelPrismaticUnaligned
    """
    @staticmethod
    def __eq__(arg1: JointModelPrismaticUnaligned, arg2: JointModelPrismaticUnaligned) -> object: 
        """
        __eq__( (JointModelPrismaticUnaligned)arg1, (JointModelPrismaticUnaligned)arg2) -> object
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @typing.overload
    def __init__(self, axis: numpy.ndarray) -> None: ...
    @typing.overload
    def __init__(self, x: float, y: float, z: float) -> None: ...
    @staticmethod
    def __ne__(arg1: JointModelPrismaticUnaligned, arg2: JointModelPrismaticUnaligned) -> object: 
        """
        __ne__( (JointModelPrismaticUnaligned)arg1, (JointModelPrismaticUnaligned)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelPrismaticUnaligned) -> object: 
        """
        __repr__( (JointModelPrismaticUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelPrismaticUnaligned) -> object: 
        """
        __str__( (JointModelPrismaticUnaligned)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataPrismaticUnaligned, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelPrismaticUnaligned)self, (JointDataPrismaticUnaligned)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataPrismaticUnaligned, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataPrismaticUnaligned: 
        """
        createData( (JointModelPrismaticUnaligned)self) -> JointDataPrismaticUnaligned :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelPrismaticUnaligned)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelPrismaticUnaligned)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelPrismaticUnaligned)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def axis(self) -> numpy.ndarray:
        """
        Translation axis of the JointModelPrismaticUnaligned.

        :type: numpy.ndarray
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelRUBX(Boost.Python.instance):
    """
    JointModelRUBX
    """
    @staticmethod
    def __eq__(arg1: JointModelRUBX, arg2: JointModelRUBX) -> object: 
        """
        __eq__( (JointModelRUBX)arg1, (JointModelRUBX)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelRUBX, arg2: JointModelRUBX) -> object: 
        """
        __ne__( (JointModelRUBX)arg1, (JointModelRUBX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelRUBX) -> object: 
        """
        __repr__( (JointModelRUBX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelRUBX) -> object: 
        """
        __str__( (JointModelRUBX)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataRUBX, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelRUBX)self, (JointDataRUBX)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataRUBX, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataRUBX: 
        """
        createData( (JointModelRUBX)self) -> JointDataRUBX :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelRUBX) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelRUBX)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelRUBX.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelRUBX)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelRUBX)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelRUBX)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelRUBY(Boost.Python.instance):
    """
    JointModelRUBY
    """
    @staticmethod
    def __eq__(arg1: JointModelRUBY, arg2: JointModelRUBY) -> object: 
        """
        __eq__( (JointModelRUBY)arg1, (JointModelRUBY)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelRUBY, arg2: JointModelRUBY) -> object: 
        """
        __ne__( (JointModelRUBY)arg1, (JointModelRUBY)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelRUBY) -> object: 
        """
        __repr__( (JointModelRUBY)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelRUBY) -> object: 
        """
        __str__( (JointModelRUBY)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataRUBY, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelRUBY)self, (JointDataRUBY)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataRUBY, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataRUBY: 
        """
        createData( (JointModelRUBY)self) -> JointDataRUBY :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelRUBY) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelRUBY)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelRUBY.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelRUBY)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelRUBY)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelRUBY)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelRUBZ(Boost.Python.instance):
    """
    JointModelRUBZ
    """
    @staticmethod
    def __eq__(arg1: JointModelRUBZ, arg2: JointModelRUBZ) -> object: 
        """
        __eq__( (JointModelRUBZ)arg1, (JointModelRUBZ)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelRUBZ, arg2: JointModelRUBZ) -> object: 
        """
        __ne__( (JointModelRUBZ)arg1, (JointModelRUBZ)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelRUBZ) -> object: 
        """
        __repr__( (JointModelRUBZ)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelRUBZ) -> object: 
        """
        __str__( (JointModelRUBZ)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataRUBZ, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelRUBZ)self, (JointDataRUBZ)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataRUBZ, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataRUBZ: 
        """
        createData( (JointModelRUBZ)self) -> JointDataRUBZ :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelRUBZ) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelRUBZ)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelRUBZ.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelRUBZ)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelRUBZ)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelRUBZ)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelRX(Boost.Python.instance):
    """
    JointModelRX
    """
    @staticmethod
    def __eq__(arg1: JointModelRX, arg2: JointModelRX) -> object: 
        """
        __eq__( (JointModelRX)arg1, (JointModelRX)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelRX, arg2: JointModelRX) -> object: 
        """
        __ne__( (JointModelRX)arg1, (JointModelRX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelRX) -> object: 
        """
        __repr__( (JointModelRX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelRX) -> object: 
        """
        __str__( (JointModelRX)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataRX, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelRX)self, (JointDataRX)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataRX, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataRX: 
        """
        createData( (JointModelRX)self) -> JointDataRX :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelRX) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelRX)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelRX.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelRX)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelRX)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelRX)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelRY(Boost.Python.instance):
    """
    JointModelRY
    """
    @staticmethod
    def __eq__(arg1: JointModelRY, arg2: JointModelRY) -> object: 
        """
        __eq__( (JointModelRY)arg1, (JointModelRY)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelRY, arg2: JointModelRY) -> object: 
        """
        __ne__( (JointModelRY)arg1, (JointModelRY)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelRY) -> object: 
        """
        __repr__( (JointModelRY)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelRY) -> object: 
        """
        __str__( (JointModelRY)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataRY, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelRY)self, (JointDataRY)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataRY, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataRY: 
        """
        createData( (JointModelRY)self) -> JointDataRY :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelRY) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelRY)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelRY.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelRY)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelRY)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelRY)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelRZ(Boost.Python.instance):
    """
    JointModelRZ
    """
    @staticmethod
    def __eq__(arg1: JointModelRZ, arg2: JointModelRZ) -> object: 
        """
        __eq__( (JointModelRZ)arg1, (JointModelRZ)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelRZ, arg2: JointModelRZ) -> object: 
        """
        __ne__( (JointModelRZ)arg1, (JointModelRZ)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelRZ) -> object: 
        """
        __repr__( (JointModelRZ)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelRZ) -> object: 
        """
        __str__( (JointModelRZ)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataRZ, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelRZ)self, (JointDataRZ)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataRZ, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataRZ: 
        """
        createData( (JointModelRZ)self) -> JointDataRZ :
            Create data associated to the joint model.
        """
    @staticmethod
    def getMotionAxis(arg1: JointModelRZ) -> numpy.ndarray: 
        """
        getMotionAxis( (JointModelRZ)arg1) -> numpy.ndarray :
            Rotation axis of the JointModelRZ.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelRZ)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelRZ)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelRZ)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelRevoluteUnaligned(Boost.Python.instance):
    """
    JointModelRevoluteUnaligned
    """
    @staticmethod
    def __eq__(arg1: JointModelRevoluteUnaligned, arg2: JointModelRevoluteUnaligned) -> object: 
        """
        __eq__( (JointModelRevoluteUnaligned)arg1, (JointModelRevoluteUnaligned)arg2) -> object
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @typing.overload
    def __init__(self, axis: numpy.ndarray) -> None: ...
    @typing.overload
    def __init__(self, x: float, y: float, z: float) -> None: ...
    @staticmethod
    def __ne__(arg1: JointModelRevoluteUnaligned, arg2: JointModelRevoluteUnaligned) -> object: 
        """
        __ne__( (JointModelRevoluteUnaligned)arg1, (JointModelRevoluteUnaligned)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelRevoluteUnaligned) -> object: 
        """
        __repr__( (JointModelRevoluteUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelRevoluteUnaligned) -> object: 
        """
        __str__( (JointModelRevoluteUnaligned)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataRevoluteUnaligned, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelRevoluteUnaligned)self, (JointDataRevoluteUnaligned)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataRevoluteUnaligned, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataRevoluteUnaligned: 
        """
        createData( (JointModelRevoluteUnaligned)self) -> JointDataRevoluteUnaligned :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelRevoluteUnaligned)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelRevoluteUnaligned)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelRevoluteUnaligned)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def axis(self) -> numpy.ndarray:
        """
        Rotation axis of the JointModelRevoluteUnaligned.

        :type: numpy.ndarray
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelRevoluteUnboundedUnaligned(Boost.Python.instance):
    """
    JointModelRevoluteUnboundedUnaligned
    """
    @staticmethod
    def __eq__(arg1: JointModelRevoluteUnboundedUnaligned, arg2: JointModelRevoluteUnboundedUnaligned) -> object: 
        """
        __eq__( (JointModelRevoluteUnboundedUnaligned)arg1, (JointModelRevoluteUnboundedUnaligned)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelRevoluteUnboundedUnaligned, arg2: JointModelRevoluteUnboundedUnaligned) -> object: 
        """
        __ne__( (JointModelRevoluteUnboundedUnaligned)arg1, (JointModelRevoluteUnboundedUnaligned)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelRevoluteUnboundedUnaligned) -> object: 
        """
        __repr__( (JointModelRevoluteUnboundedUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelRevoluteUnboundedUnaligned) -> object: 
        """
        __str__( (JointModelRevoluteUnboundedUnaligned)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataRevoluteUnboundedUnalignedTpl, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelRevoluteUnboundedUnaligned)self, (JointDataRevoluteUnboundedUnalignedTpl)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataRevoluteUnboundedUnalignedTpl, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataRevoluteUnboundedUnalignedTpl: 
        """
        createData( (JointModelRevoluteUnboundedUnaligned)self) -> JointDataRevoluteUnboundedUnalignedTpl :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelRevoluteUnboundedUnaligned)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelRevoluteUnboundedUnaligned)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelRevoluteUnboundedUnaligned)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelSpherical(Boost.Python.instance):
    """
    JointModelSpherical
    """
    @staticmethod
    def __eq__(arg1: JointModelSpherical, arg2: JointModelSpherical) -> object: 
        """
        __eq__( (JointModelSpherical)arg1, (JointModelSpherical)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelSpherical, arg2: JointModelSpherical) -> object: 
        """
        __ne__( (JointModelSpherical)arg1, (JointModelSpherical)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelSpherical) -> object: 
        """
        __repr__( (JointModelSpherical)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelSpherical) -> object: 
        """
        __str__( (JointModelSpherical)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataSpherical, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelSpherical)self, (JointDataSpherical)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataSpherical, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataSpherical: 
        """
        createData( (JointModelSpherical)self) -> JointDataSpherical :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelSpherical)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelSpherical)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelSpherical)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelSphericalZYX(Boost.Python.instance):
    """
    JointModelSphericalZYX
    """
    @staticmethod
    def __eq__(arg1: JointModelSphericalZYX, arg2: JointModelSphericalZYX) -> object: 
        """
        __eq__( (JointModelSphericalZYX)arg1, (JointModelSphericalZYX)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelSphericalZYX, arg2: JointModelSphericalZYX) -> object: 
        """
        __ne__( (JointModelSphericalZYX)arg1, (JointModelSphericalZYX)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelSphericalZYX) -> object: 
        """
        __repr__( (JointModelSphericalZYX)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelSphericalZYX) -> object: 
        """
        __str__( (JointModelSphericalZYX)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataSphericalZYX, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelSphericalZYX)self, (JointDataSphericalZYX)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataSphericalZYX, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataSphericalZYX: 
        """
        createData( (JointModelSphericalZYX)self) -> JointDataSphericalZYX :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelSphericalZYX)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelSphericalZYX)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelSphericalZYX)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelTranslation(Boost.Python.instance):
    """
    JointModelTranslation
    """
    @staticmethod
    def __eq__(arg1: JointModelTranslation, arg2: JointModelTranslation) -> object: 
        """
        __eq__( (JointModelTranslation)arg1, (JointModelTranslation)arg2) -> object
        """
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(arg1: JointModelTranslation, arg2: JointModelTranslation) -> object: 
        """
        __ne__( (JointModelTranslation)arg1, (JointModelTranslation)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelTranslation) -> object: 
        """
        __repr__( (JointModelTranslation)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelTranslation) -> object: 
        """
        __str__( (JointModelTranslation)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataTranslation, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelTranslation)self, (JointDataTranslation)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataTranslation, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataTranslation: 
        """
        createData( (JointModelTranslation)self) -> JointDataTranslation :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelTranslation)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelTranslation)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelTranslation)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class JointModelUniversal(Boost.Python.instance):
    """
    JointModelUniversal
    """
    @staticmethod
    def __eq__(arg1: JointModelUniversal, arg2: JointModelUniversal) -> object: 
        """
        __eq__( (JointModelUniversal)arg1, (JointModelUniversal)arg2) -> object
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None
        """
    @typing.overload
    def __init__(self, axis1: numpy.ndarray, axis2: numpy.ndarray) -> None: ...
    @typing.overload
    def __init__(self, x1: float, y1: float, z1: float, x2: float, y2: float, z2: float) -> None: ...
    @staticmethod
    def __ne__(arg1: JointModelUniversal, arg2: JointModelUniversal) -> object: 
        """
        __ne__( (JointModelUniversal)arg1, (JointModelUniversal)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: JointModelUniversal) -> object: 
        """
        __repr__( (JointModelUniversal)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: JointModelUniversal) -> object: 
        """
        __str__( (JointModelUniversal)arg1) -> object
        """
    @typing.overload
    def calc(self, jdata: JointDataUniversal, q: numpy.ndarray) -> None: 
        """
        calc( (JointModelUniversal)self, (JointDataUniversal)jdata, (numpy.ndarray)q) -> None
        """
    @typing.overload
    def calc(self, jdata: JointDataUniversal, q: numpy.ndarray, v: numpy.ndarray) -> None: ...
    @staticmethod
    def classname() -> str: 
        """
        classname() -> str
        """
    def createData(self) -> JointDataUniversal: 
        """
        createData( (JointModelUniversal)self) -> JointDataUniversal :
            Create data associated to the joint model.
        """
    def hasSameIndexes(self, other: object) -> bool: 
        """
        hasSameIndexes( (JointModelUniversal)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    def setIndexes(self, joint_id: int, idx_q: int, idx_v: int) -> None: 
        """
        setIndexes( (JointModelUniversal)self, (int)joint_id, (int)idx_q, (int)idx_v) -> None
        """
    def shortname(self) -> str: 
        """
        shortname( (JointModelUniversal)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def axis1(self) -> numpy.ndarray:
        """
        First rotation axis of the JointModelUniversal.

        :type: numpy.ndarray
        """
    @property
    def axis2(self) -> numpy.ndarray:
        """
        Second rotation axis of the JointModelUniversal.

        :type: numpy.ndarray
        """
    @property
    def hasConfigurationLimit(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits.

        :type: StdVec_Bool
        """
    @property
    def hasConfigurationLimitInTangent(self) -> Sequence[bool]:
        """
        Return vector of boolean if joint has configuration limits in tangent space.

        :type: StdVec_Bool
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def idx_q(self) -> int:
        """
        :type: int
        """
    @property
    def idx_v(self) -> int:
        """
        :type: int
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class LanczosDecomposition(Boost.Python.instance):
    """
    Lanczos decomposition.
    """
    def Qs(self) -> object: 
        """
        Qs( (LanczosDecomposition)self) -> object :
            Returns the orthogonal basis associated with the Lanczos decomposition.
        """
    def Ts(self) -> TridiagonalSymmetricMatrix: 
        """
        Ts( (LanczosDecomposition)self) -> TridiagonalSymmetricMatrix :
            Returns the tridiagonal matrix associated with the Lanczos decomposition.
        """
    @staticmethod
    def __eq__(arg1: LanczosDecomposition, arg2: LanczosDecomposition) -> object: 
        """
        __eq__( (LanczosDecomposition)arg1, (LanczosDecomposition)arg2) -> object
        """
    def __init__(self, mat: numpy.ndarray, decomposition_size: int) -> None: 
        """
        __init__( (object)self, (numpy.ndarray)mat, (int)decomposition_size) -> None :
            Default constructor from a given matrix and a given decomposition size.
        """
    @staticmethod
    def __ne__(arg1: LanczosDecomposition, arg2: LanczosDecomposition) -> object: 
        """
        __ne__( (LanczosDecomposition)arg1, (LanczosDecomposition)arg2) -> object
        """
    def compute(self, mat: numpy.ndarray) -> None: 
        """
        compute( (LanczosDecomposition)self, (numpy.ndarray)mat) -> None :
            Computes the Lanczos decomposition for the given input matrix.
        """
    def computeDecompositionResidual(self, mat: numpy.ndarray) -> numpy.ndarray: 
        """
        computeDecompositionResidual( (LanczosDecomposition)self, (numpy.ndarray)mat) -> numpy.ndarray :
            Computes the residual associated with the decomposition, namely, the quantity 
$ A Q_s - Q_s T_s 
$
        """
    def rank(self) -> int: 
        """
        rank( (LanczosDecomposition)self) -> int :
            Returns the rank of the decomposition.
        """
    pass

class LieGroup(Boost.Python.instance):
    @staticmethod
    def __eq__(arg1: LieGroup, arg2: LieGroup) -> object: 
        """
        __eq__( (LieGroup)arg1, (LieGroup)arg2) -> object
        """
    @staticmethod
    def __imul__(arg1: object, arg2: LieGroup) -> object: 
        """
        __imul__( (object)arg1, (LieGroup)arg2) -> object
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :
            Default constructor
        """
    @staticmethod
    def __mul__(arg1: LieGroup, arg2: LieGroup) -> object: 
        """
        __mul__( (LieGroup)arg1, (LieGroup)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def dDifference(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray, arg4: ArgumentPosition) -> numpy.ndarray: 
        """
        dDifference( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (ArgumentPosition)arg4) -> numpy.ndarray
        """
    @staticmethod
    @typing.overload
    def dDifference(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray, arg4: ArgumentPosition, arg5: int, arg6: numpy.ndarray) -> numpy.ndarray: ...
    @staticmethod
    @typing.overload
    def dDifference(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray, arg4: ArgumentPosition, arg5: numpy.ndarray, arg6: int) -> numpy.ndarray: ...
    @staticmethod
    def dIntegrate(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray, arg4: ArgumentPosition) -> numpy.ndarray: 
        """
        dIntegrate( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (ArgumentPosition)arg4) -> numpy.ndarray
        """
    @staticmethod
    def dIntegrateTransport(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray, arg4: numpy.ndarray, arg5: ArgumentPosition) -> numpy.ndarray: 
        """
        dIntegrateTransport( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (numpy.ndarray)arg4, (ArgumentPosition)arg5) -> numpy.ndarray
        """
    @staticmethod
    @typing.overload
    def dIntegrate_dq(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray) -> numpy.ndarray: 
        """
        dIntegrate_dq( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> numpy.ndarray
        """
    @staticmethod
    @typing.overload
    def dIntegrate_dq(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray, arg4: int, arg5: numpy.ndarray) -> numpy.ndarray: ...
    @staticmethod
    @typing.overload
    def dIntegrate_dq(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray, arg4: numpy.ndarray, arg5: int) -> numpy.ndarray: ...
    @staticmethod
    @typing.overload
    def dIntegrate_dv(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray) -> numpy.ndarray: 
        """
        dIntegrate_dv( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> numpy.ndarray
        """
    @staticmethod
    @typing.overload
    def dIntegrate_dv(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray, arg4: int, arg5: numpy.ndarray) -> numpy.ndarray: ...
    @staticmethod
    @typing.overload
    def dIntegrate_dv(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray, arg4: numpy.ndarray, arg5: int) -> numpy.ndarray: ...
    @staticmethod
    def difference(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray) -> numpy.ndarray: 
        """
        difference( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> numpy.ndarray
        """
    @staticmethod
    def distance(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray) -> float: 
        """
        distance( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> float
        """
    @staticmethod
    def integrate(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray) -> numpy.ndarray: 
        """
        integrate( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> numpy.ndarray
        """
    @staticmethod
    def interpolate(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray, arg4: float) -> numpy.ndarray: 
        """
        interpolate( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (float)arg4) -> numpy.ndarray
        """
    @staticmethod
    def normalize(arg1: LieGroup, arg2: numpy.ndarray) -> None: 
        """
        normalize( (LieGroup)arg1, (numpy.ndarray)arg2) -> None
        """
    @staticmethod
    def random(arg1: LieGroup) -> numpy.ndarray: 
        """
        random( (LieGroup)arg1) -> numpy.ndarray
        """
    @staticmethod
    def randomConfiguration(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray) -> numpy.ndarray: 
        """
        randomConfiguration( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> numpy.ndarray
        """
    @staticmethod
    def squaredDistance(arg1: LieGroup, arg2: numpy.ndarray, arg3: numpy.ndarray) -> float: 
        """
        squaredDistance( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> float
        """
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def neutral(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def nq(self) -> int:
        """
        :type: int
        """
    @property
    def nv(self) -> int:
        """
        :type: int
        """
    pass

class LogCholeskyParameters(Boost.Python.instance):
    """
    This class represents log Cholesky parameters.

    Supported operations ...
    """
    @staticmethod
    @typing.overload
    def __array__(arg1: LogCholeskyParameters) -> numpy.ndarray: 
        """
        __array__( (LogCholeskyParameters)arg1) -> numpy.ndarray
        """
    @typing.overload
    def __array__(self, dtype: object = None, copy: object = None) -> numpy.ndarray: ...
    def __copy__(self) -> LogCholeskyParameters: 
        """
        __copy__( (LogCholeskyParameters)self) -> LogCholeskyParameters :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> LogCholeskyParameters: 
        """
        __deepcopy__( (LogCholeskyParameters)self, (dict)memo) -> LogCholeskyParameters :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __getinitargs__(arg1: LogCholeskyParameters) -> tuple: 
        """
        __getinitargs__( (LogCholeskyParameters)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, clone: LogCholeskyParameters) -> object: 
        """
        __init__( (object)arg1, (numpy.ndarray)log_cholesky_parameters) -> object :
            Initialize from log cholesky parameters.
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, log_cholesky_parameters: numpy.ndarray) -> object: ...
    @typing.overload
    def __init__(self, clone: LogCholeskyParameters) -> None: ...
    @staticmethod
    def __repr__(arg1: LogCholeskyParameters) -> object: 
        """
        __repr__( (LogCholeskyParameters)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: LogCholeskyParameters) -> object: 
        """
        __str__( (LogCholeskyParameters)arg1) -> object
        """
    def calculateJacobian(self) -> numpy.ndarray: 
        """
        calculateJacobian( (LogCholeskyParameters)self) -> numpy.ndarray :
            Calculates the Jacobian of the log Cholesky parameters.
        """
    @staticmethod
    def cast(arg1: LogCholeskyParameters) -> LogCholeskyParameters: 
        """
        cast( (LogCholeskyParameters)arg1) -> LogCholeskyParameters :
            Returns a cast of *this.
        """
    def copy(self) -> LogCholeskyParameters: 
        """
        copy( (LogCholeskyParameters)self) -> LogCholeskyParameters :
            Returns a copy of *this.
        """
    def toDynamicParameters(self) -> numpy.ndarray: 
        """
        toDynamicParameters( (LogCholeskyParameters)self) -> numpy.ndarray :
            Returns the dynamic parameters representation.
        """
    def toInertia(self) -> Inertia: 
        """
        toInertia( (LogCholeskyParameters)self) -> Inertia :
            Returns the Inertia representation.
        """
    def toPseudoInertia(self) -> PseudoInertia: 
        """
        toPseudoInertia( (LogCholeskyParameters)self) -> PseudoInertia :
            Returns the Pseudo Inertia representation.
        """
    @property
    def parameters(self) -> numpy.ndarray:
        """
        Log Cholesky parameters.

        :type: numpy.ndarray
        """
    __safe_for_unpickling__ = True
    pass

class Model(Boost.Python.instance):
    """
    Articulated Rigid Body model
    """
    def __copy__(self) -> Model: 
        """
        __copy__( (Model)self) -> Model :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> Model: 
        """
        __deepcopy__( (Model)self, (dict)memo) -> Model :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: Model, arg2: Model) -> object: 
        """
        __eq__( (Model)arg1, (Model)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(arg1: Model) -> tuple: 
        """
        __getinitargs__( (Model)arg1) -> tuple
        """
    @staticmethod
    def __getstate__(arg1: Model) -> tuple: 
        """
        __getstate__( (Model)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, clone: Model) -> object: 
        """
        __init__( (object)self) -> None :
            Default constructor. Constructs an empty model.
        """
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, clone: Model) -> None: ...
    @staticmethod
    def __ne__(arg1: Model, arg2: Model) -> object: 
        """
        __ne__( (Model)arg1, (Model)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: Model) -> object: 
        """
        __repr__( (Model)arg1) -> object
        """
    @staticmethod
    def __setstate__(arg1: Model, arg2: tuple) -> None: 
        """
        __setstate__( (Model)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def __str__(arg1: Model) -> object: 
        """
        __str__( (Model)arg1) -> object
        """
    def addBodyFrame(self, body_name: str, parentJoint: int, body_placement: SE3, previous_frame: int) -> int: 
        """
        addBodyFrame( (Model)self, (str)body_name, (int)parentJoint, (SE3)body_placement, (int)previous_frame) -> int :
            add a body to the frame tree
        """
    def addFrame(self, frame_: Frame, append_inertia: bool = True) -> int: 
        """
        addFrame( (Model)self, (Frame)frame [, (bool)append_inertia=True]) -> int :
            Add a frame to the vector of frames. If append_inertia set to True, the inertia value contained in frame will be added to the inertia supported by the parent joint.
        """
    @typing.overload
    def addJoint(self, parent_id: int, joint_model: JointModel, joint_placement: SE3, joint_name: str) -> int: 
        """
        addJoint( (Model)self, (int)parent_id, (JointModel)joint_model, (SE3)joint_placement, (str)joint_name) -> int :
            Adds a joint to the kinematic tree. The joint is defined by its placement relative to its parent joint and its name.
        """
    @typing.overload
    def addJoint(self, parent_id: int, joint_model: JointModel, joint_placement: SE3, joint_name: str, max_effort: numpy.ndarray, max_velocity: numpy.ndarray, min_config: numpy.ndarray, max_config: numpy.ndarray) -> int: ...
    @typing.overload
    def addJoint(self, parent_id: int, joint_model: JointModel, joint_placement: SE3, joint_name: str, max_effort: numpy.ndarray, max_velocity: numpy.ndarray, min_config: numpy.ndarray, max_config: numpy.ndarray, friction: numpy.ndarray, damping: numpy.ndarray) -> int: ...
    def addJointFrame(self, joint_id_: int, frame_id: int = 0) -> int: 
        """
        addJointFrame( (Model)self, (int)joint_id [, (int)frame_id=0]) -> int :
            Add the joint provided by its joint_id as a frame to the frame tree.
            The frame_id may be optionally provided.
        """
    def appendBodyToJoint(self, joint_id: int, body_inertia: Inertia, body_placement: SE3) -> None: 
        """
        appendBodyToJoint( (Model)self, (int)joint_id, (Inertia)body_inertia, (SE3)body_placement) -> None :
            Appends a body to the joint given by its index. The body is defined by its inertia, its relative placement regarding to the joint and its name.
        """
    @staticmethod
    def cast(arg1: Model) -> Model: 
        """
        cast( (Model)arg1) -> Model :
            Returns a cast of *this.
        """
    def check(self, data: Data) -> bool: 
        """
        check( (Model)self, (Data)data) -> bool :
            Check consistency of data wrt model.
        """
    def copy(self) -> Model: 
        """
        copy( (Model)self) -> Model :
            Returns a copy of *this.
        """
    def createData(self) -> Data: 
        """
        createData( (Model)self) -> Data :
            Create a Data object for the given model.
        """
    def existBodyName(self, name: str) -> bool: 
        """
        existBodyName( (Model)self, (str)name) -> bool :
            Check if a frame of type BODY exists, given its name
        """
    def existFrame(self, name_: str, type: FrameType = FrameType(31)) -> bool: 
        """
        existFrame( (Model)self, (str)name [, (FrameType)type=pinocchio.pinocchio_pywrap_default.FrameType(31)]) -> bool :
            Returns true if the frame given by its name exists inside the Model with the given type.
        """
    def existJointName(self, name: str) -> bool: 
        """
        existJointName( (Model)self, (str)name) -> bool :
            Check if a joint given by its name exists
        """
    def getBodyId(self, name: str) -> int: 
        """
        getBodyId( (Model)self, (str)name) -> int :
            Return the index of a frame of type BODY given by its name
        """
    def getFrameId(self, name_: str, type: FrameType = FrameType(31)) -> int: 
        """
        getFrameId( (Model)self, (str)name [, (FrameType)type=pinocchio.pinocchio_pywrap_default.FrameType(31)]) -> int :
            Returns the index of the frame given by its name and its type.If the frame is not in the frames vector, it returns the current size of the frames vector.
        """
    def getJointId(self, name: str) -> int: 
        """
        getJointId( (Model)self, (str)name) -> int :
            Return the index of a joint given by its name
        """
    def hasConfigurationLimit(self) -> StdVec_Bool: 
        """
        hasConfigurationLimit( (Model)self) -> StdVec_Bool :
            Returns list of boolean if joints have configuration limit.
        """
    def hasConfigurationLimitInTangent(self) -> StdVec_Bool: 
        """
        hasConfigurationLimitInTangent( (Model)self) -> StdVec_Bool :
            Returns list of boolean if joints have configuration limit in tangent space  .
        """
    @typing.overload
    def loadFromBinary(self, buffer: StaticBuffer) -> None: 
        """
        loadFromBinary( (Model)self, (str)filename) -> None :
            Loads *this from a binary file.
        """
    @typing.overload
    def loadFromBinary(self, buffer: StreamBuffer) -> None: ...
    @typing.overload
    def loadFromBinary(self, filename: str) -> None: ...
    def loadFromString(self, string: str) -> None: 
        """
        loadFromString( (Model)self, (str)string) -> None :
            Parses from the input string the content of the current object.
        """
    def loadFromText(self, filename: str) -> None: 
        """
        loadFromText( (Model)self, (str)filename) -> None :
            Loads *this from a text file.
        """
    def loadFromXML(self, filename: str, tag_name: str) -> None: 
        """
        loadFromXML( (Model)self, (str)filename, (str)tag_name) -> None :
            Loads *this from a XML file.
        """
    @typing.overload
    def saveToBinary(self, buffer: StaticBuffer) -> None: 
        """
        saveToBinary( (Model)self, (str)filename) -> None :
            Saves *this inside a binary file.
        """
    @typing.overload
    def saveToBinary(self, buffer: StreamBuffer) -> None: ...
    @typing.overload
    def saveToBinary(self, filename: str) -> None: ...
    def saveToString(self) -> str: 
        """
        saveToString( (Model)self) -> str :
            Parses the current object to a string.
        """
    def saveToText(self, filename: str) -> None: 
        """
        saveToText( (Model)self, (str)filename) -> None :
            Saves *this inside a text file.
        """
    @staticmethod
    def saveToXML(arg1: Model, filename: str, tag_name: str) -> None: 
        """
        saveToXML( (Model)arg1, (str)filename, (str)tag_name) -> None :
            Saves *this inside a XML file.
        """
    @property
    def armature(self) -> numpy.ndarray:
        """
        Armature vector.

        :type: numpy.ndarray
        """
    @property
    def children(self) -> Sequence[Sequence[int]]:
        """
        Vector of children index. Chidren of the *i*th joint, denoted *mu(i)* corresponds to the set (i==parents[k] for k in mu(i)).

        :type: Sequence[Sequence[int]]
        """
    @property
    def damping(self) -> numpy.ndarray:
        """
        Vector of joint damping parameters.

        :type: numpy.ndarray
        """
    @property
    def effortLimit(self) -> numpy.ndarray:
        """
        Joint max effort.

        :type: numpy.ndarray
        """
    @property
    def frames(self) -> Sequence[Frame]:
        """
        Vector of frames contained in the model.

        :type: Sequence[Frame]
        """
    @property
    def friction(self) -> numpy.ndarray:
        """
        Vector of joint friction parameters.

        :type: numpy.ndarray
        """
    @property
    def gravity(self) -> Motion:
        """
        Motion vector corresponding to the gravity field expressed in the world Frame.

        :type: Motion
        """
    @property
    def idx_qs(self) -> numpy.ndarray:
        """
        Vector of starting index of the *i*th  joint in the configuration space.

        :type: numpy.ndarray
        """
    @property
    def idx_vs(self) -> numpy.ndarray:
        """
        Starting index of the *i*th joint in the tangent configuration space.

        :type: numpy.ndarray
        """
    @property
    def inertias(self) -> Sequence[Inertia]:
        """
        Vector of spatial inertias supported by each joint.

        :type: Sequence[Inertia]
        """
    @property
    def jointPlacements(self) -> Sequence[SE3]:
        """
        Vector of joint placements: placement of a joint *i* wrt its parent joint frame.

        :type: Sequence[SE3]
        """
    @property
    def joints(self) -> Sequence[JointModel]:
        """
        Vector of joint models.

        :type: Sequence[JointModel]
        """
    @property
    def lowerPositionLimit(self) -> numpy.ndarray:
        """
        Limit for joint lower position.

        :type: numpy.ndarray
        """
    @property
    def name(self) -> str:
        """
        Name of the model.

        :type: str
        """
    @property
    def names(self) -> Sequence[str]:
        """
        Name of the joints.

        :type: Sequence[str]
        """
    @property
    def nbodies(self) -> int:
        """
        Number of bodies.

        :type: int
        """
    @property
    def nframes(self) -> int:
        """
        Number of frames.

        :type: int
        """
    @property
    def njoints(self) -> int:
        """
        Number of joints.

        :type: int
        """
    @property
    def nq(self) -> int:
        """
        Dimension of the configuration vector representation.

        :type: int
        """
    @property
    def nqs(self) -> int:
        """
        Vector of dimension of the  joint configuration subspace.

        :type: int
        """
    @property
    def nv(self) -> int:
        """
        Dimension of the velocity vector space.

        :type: int
        """
    @property
    def nvs(self) -> int:
        """
        Dimension of the *i*th joint tangent subspace.

        :type: int
        """
    @property
    def parents(self) -> numpy.ndarray:
        """
        Vector of parent joint indexes. The parent of joint *i*, denoted *li*, corresponds to li==parents[i].

        :type: numpy.ndarray
        """
    @property
    def referenceConfigurations(self) -> Mapping[str, numpy.ndarray]:
        """
        Map of reference configurations, indexed by user given names.

        :type: Mapping[str, numpy.ndarray]
        """
    @property
    def rotorGearRatio(self) -> numpy.ndarray:
        """
        Vector of rotor gear ratio parameters.

        :type: numpy.ndarray
        """
    @property
    def rotorInertia(self) -> numpy.ndarray:
        """
        Vector of rotor inertia parameters.

        :type: numpy.ndarray
        """
    @property
    def subtrees(self) -> Sequence[Sequence[int]]:
        """
        Vector of subtrees. subtree[j] corresponds to the subtree supported by the joint j.

        :type: Sequence[Sequence[int]]
        """
    @property
    def supports(self) -> Sequence[Sequence[int]]:
        """
        Vector of supports. supports[j] corresponds to the list of joints on the path between
        the current *j* to the root of the kinematic tree.

        :type: Sequence[Sequence[int]]
        """
    @property
    def upperPositionLimit(self) -> numpy.ndarray:
        """
        Limit for joint upper position.

        :type: numpy.ndarray
        """
    @property
    def velocityLimit(self) -> numpy.ndarray:
        """
        Joint max velocity.

        :type: numpy.ndarray
        """
    __getstate_manages_dict__ = True
    __safe_for_unpickling__ = True
    gravity981: numpy.ndarray # value = array([ 0.  ,  0.  , -9.81])
    pass

class Motion(Boost.Python.instance):
    """
    Motion vectors, in se3 == M^6.

    Supported operations ...
    """
    @staticmethod
    def Random() -> Motion: 
        """
        Random() -> Motion :
            Returns a random Motion.
        """
    @staticmethod
    def Zero() -> Motion: 
        """
        Zero() -> Motion :
            Returns a zero Motion.
        """
    @staticmethod
    def __add__(arg1: Motion, arg2: Motion) -> object: 
        """
        __add__( (Motion)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __array__(arg1: Motion) -> object: 
        """
        __array__( (Motion)arg1) -> object
        """
    @typing.overload
    def __array__(self, dtype: object = None, copy: object = None) -> object: ...
    def __copy__(self) -> Motion: 
        """
        __copy__( (Motion)self) -> Motion :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> Motion: 
        """
        __deepcopy__( (Motion)self, (dict)memo) -> Motion :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: Motion, arg2: Motion) -> object: 
        """
        __eq__( (Motion)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(arg1: Motion) -> tuple: 
        """
        __getinitargs__( (Motion)arg1) -> tuple
        """
    @staticmethod
    def __iadd__(arg1: object, arg2: Motion) -> object: 
        """
        __iadd__( (object)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, clone: Motion) -> object: 
        """
        __init__( (object)self) -> None :
            Default constructor
        """
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None: ...
    @typing.overload
    def __init__(self, clone: Motion) -> None: ...
    @typing.overload
    def __init__(self, linear: numpy.ndarray, angular: numpy.ndarray) -> None: ...
    @staticmethod
    def __isub__(arg1: object, arg2: Motion) -> object: 
        """
        __isub__( (object)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __mul__(arg1: Motion, arg2: float) -> object: 
        """
        __mul__( (Motion)arg1, (float)arg2) -> object
        """
    @staticmethod
    def __ne__(arg1: Motion, arg2: Motion) -> object: 
        """
        __ne__( (Motion)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __neg__(arg1: Motion) -> object: 
        """
        __neg__( (Motion)arg1) -> object
        """
    @staticmethod
    def __repr__(arg1: Motion) -> object: 
        """
        __repr__( (Motion)arg1) -> object
        """
    @staticmethod
    def __rmul__(arg1: Motion, arg2: float) -> object: 
        """
        __rmul__( (Motion)arg1, (float)arg2) -> object
        """
    @staticmethod
    def __str__(arg1: Motion) -> object: 
        """
        __str__( (Motion)arg1) -> object
        """
    @staticmethod
    def __sub__(arg1: Motion, arg2: Motion) -> object: 
        """
        __sub__( (Motion)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __truediv__(arg1: Motion, arg2: float) -> object: 
        """
        __truediv__( (Motion)arg1, (float)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __xor__(arg1: Motion, arg2: Force) -> object: 
        """
        __xor__( (Motion)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __xor__(arg1: Motion, arg2: Motion) -> object: ...
    @staticmethod
    def cast(arg1: Motion) -> Motion: 
        """
        cast( (Motion)arg1) -> Motion :
            Returns a cast of *this.
        """
    def copy(self) -> Motion: 
        """
        copy( (Motion)self) -> Motion :
            Returns a copy of *this.
        """
    @typing.overload
    def cross(self, f: Force) -> Force: 
        """
        cross( (Motion)self, (Motion)m) -> Motion :
            Action of *this onto another Motion m. Returns ¨*this x m.
        """
    @typing.overload
    def cross(self, m: Motion) -> Motion: ...
    def dot(self, f: object) -> float: 
        """
        dot( (Motion)self, (object)f) -> float :
            Dot product between *this and a Force f.
        """
    def isApprox(self, other_: Motion, prec: float = 1e-12) -> bool: 
        """
        isApprox( (Motion)self, (Motion)other [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to other, within the precision given by prec.
        """
    def isZero(self, prec: float = 1e-12) -> bool: 
        """
        isZero( (Motion)self [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to the zero Motion, within the precision given by prec.
        """
    def se3Action(self, M: SE3) -> Motion: 
        """
        se3Action( (Motion)self, (SE3)M) -> Motion :
            Returns the result of the action of M on *this.
        """
    def se3ActionInverse(self, M: SE3) -> Motion: 
        """
        se3ActionInverse( (Motion)self, (SE3)M) -> Motion :
            Returns the result of the action of the inverse of M on *this.
        """
    def setRandom(self) -> None: 
        """
        setRandom( (Motion)self) -> None :
            Set the linear and angular components of *this to random values.
        """
    def setZero(self) -> None: 
        """
        setZero( (Motion)self) -> None :
            Set the linear and angular components of *this to zero.
        """
    @property
    def action(self) -> FloatMat6:
        """
        Returns the action matrix of *this (acting on Motion).

        :type: FloatMat6
        """
    @property
    def angular(self) -> FloatVec3:
        """
        Angular part of a *this, corresponding to the angular velocity in case of a Spatial velocity.

        :type: FloatVec3
        """
    @property
    def dualAction(self) -> FloatMat6:
        """
        Returns the dual action matrix of *this (acting on Force).

        :type: FloatMat6
        """
    @property
    def homogeneous(self) -> FloatMat4:
        """
        Equivalent homogeneous representation of the Motion vector

        :type: FloatMat4
        """
    @property
    def linear(self) -> FloatVec3:
        """
        Linear part of a *this, corresponding to the linear velocity in case of a Spatial velocity.

        :type: FloatVec3
        """
    @property
    def np(self) -> FloatVec6:
        """
        :type: FloatVec6
        """
    @property
    def vector(self) -> FloatVec6:
        """
        Returns the components of *this as a 6d vector.

        :type: FloatVec6
        """
    __safe_for_unpickling__ = True
    pass

class PGSContactSolver(Boost.Python.instance):
    """
    Projected Gauss Siedel solver for contact dynamics.
    """
    def __init__(self, problem_dim: int) -> None: 
        """
        __init__( (object)self, (int)problem_dim) -> None :
            Default constructor.
        """
    def getAbsoluteConvergenceResidual(self) -> float: 
        """
        getAbsoluteConvergenceResidual( (PGSContactSolver)self) -> float :
            Returns the value of the absolute residual value corresponding to the contact complementary conditions.
        """
    def getAbsolutePrecision(self) -> float: 
        """
        getAbsolutePrecision( (PGSContactSolver)self) -> float :
            Get the absolute precision requested.
        """
    def getIterationCount(self) -> int: 
        """
        getIterationCount( (PGSContactSolver)self) -> int :
            Get the number of iterations achieved by the PGS algorithm.
        """
    def getMaxIterations(self) -> int: 
        """
        getMaxIterations( (PGSContactSolver)self) -> int :
            Get the maximum number of iterations allowed.
        """
    def getRelativeConvergenceResidual(self) -> float: 
        """
        getRelativeConvergenceResidual( (PGSContactSolver)self) -> float :
            Returns the value of the relative residual value corresponding to the difference between two successive iterates (infinity norms).
        """
    def getRelativePrecision(self) -> float: 
        """
        getRelativePrecision( (PGSContactSolver)self) -> float :
            Get the relative precision requested.
        """
    def setAbsolutePrecision(self, absolute_precision: float) -> None: 
        """
        setAbsolutePrecision( (PGSContactSolver)self, (float)absolute_precision) -> None :
            Set the absolute precision for the problem.
        """
    def setMaxIterations(self, max_it: int) -> None: 
        """
        setMaxIterations( (PGSContactSolver)self, (int)max_it) -> None :
            Set the maximum number of iterations.
        """
    def setRelativePrecision(self, relative_precision: float) -> None: 
        """
        setRelativePrecision( (PGSContactSolver)self, (float)relative_precision) -> None :
            Set the relative precision for the problem.
        """
    @typing.overload
    def solve(self, G: csc_matrix, g: numpy.ndarray, cones: StdVec_CoulombFrictionCone, x_: numpy.ndarray, over_relax: float = 1.0) -> bool: 
        """
        solve( (PGSContactSolver)self, (numpy.ndarray)G, (numpy.ndarray)g, (StdVec_CoulombFrictionCone)cones, (numpy.ndarray)x [, (float)over_relax=1.0]) -> bool :
            Solve the constrained conic problem composed of problem data (G,g,cones) and starting from the initial guess.
        """
    @typing.overload
    def solve(self, G: numpy.ndarray, g: numpy.ndarray, cones: StdVec_CoulombFrictionCone, x_: numpy.ndarray, over_relax: float = 1.0) -> bool: ...
    __instance_size__ = 104
    pass

class PowerIterationAlgo(Boost.Python.instance):
    def __init__(self, size_: int, max_it: int = 10, rel_tol: float = 1e-08) -> None: 
        """
        __init__( (object)self, (int)size [, (int)max_it=10 [, (float)rel_tol=1e-08]]) -> None :
            Default constructor
        """
    @staticmethod
    def lowest(arg1: PowerIterationAlgo, self_: numpy.ndarray, compute_largest: bool = True) -> None: 
        """
        lowest( (PowerIterationAlgo)arg1, (numpy.ndarray)self [, (bool)compute_largest=True]) -> None
        """
    def reset(self) -> None: 
        """
        reset( (PowerIterationAlgo)self) -> None
        """
    @staticmethod
    def run(arg1: PowerIterationAlgo, self: numpy.ndarray) -> None: 
        """
        run( (PowerIterationAlgo)arg1, (numpy.ndarray)self) -> None
        """
    @property
    def convergence_criteria(self) -> float:
        """
        :type: float
        """
    @property
    def it(self) -> int:
        """
        :type: int
        """
    @property
    def largest_eigen_value(self) -> float:
        """
        :type: float
        """
    @property
    def lowest_eigen_value(self) -> float:
        """
        :type: float
        """
    @property
    def lowest_eigen_vector(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def max_it(self) -> int:
        """
        :type: int
        """
    @property
    def principal_eigen_vector(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def rel_tol(self) -> float:
        """
        :type: float
        """
    __instance_size__ = 120
    pass

class ProximalSettings(Boost.Python.instance):
    """
    Structure containing all the settings parameters for proximal algorithms.
    """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None :
            Default constructor.
        """
    @typing.overload
    def __init__(self, absolute_accuracy: float, relative_accuracy: float, mu: float, max_iter: int) -> None: ...
    @typing.overload
    def __init__(self, accuracy: float, mu: float, max_iter: int) -> None: ...
    @staticmethod
    def __repr__(arg1: ProximalSettings) -> str: 
        """
        __repr__( (ProximalSettings)arg1) -> str
        """
    @property
    def absolute_accuracy(self) -> float:
        """
        Absolute proximal accuracy.

        :type: float
        """
    @property
    def absolute_residual(self) -> float:
        """
        Absolute residual.

        :type: float
        """
    @property
    def iter(self) -> int:
        """
        Final number of iteration of the algorithm when it has converged or reached the maximal number of allowed iterations.

        :type: int
        """
    @property
    def max_iter(self) -> int:
        """
        Maximal number of iterations.

        :type: int
        """
    @property
    def mu(self) -> float:
        """
        Regularization parameter of the Proximal algorithms.

        :type: float
        """
    @property
    def relative_accuracy(self) -> float:
        """
        Relative proximal accuracy between two iterates.

        :type: float
        """
    @property
    def relative_residual(self) -> float:
        """
        Relatice residual between two iterates.

        :type: float
        """
    pass

class PseudoInertia(Boost.Python.instance):
    """
    This class represents a pseudo inertia matrix and it is defined by its mass, vector part, and 3x3 matrix part.

    Supported operations ...
    """
    @staticmethod
    def FromDynamicParameters(dynamic_parameters: numpy.ndarray) -> PseudoInertia: 
        """
        FromDynamicParameters( (numpy.ndarray)dynamic_parameters) -> PseudoInertia :
            Builds a pseudo inertia matrix from a vector of dynamic parameters.
            The parameters are given as dynamic_parameters = [m, h_x, h_y, h_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T.
        """
    @staticmethod
    def FromInertia(inertia: Inertia) -> PseudoInertia: 
        """
        FromInertia( (Inertia)inertia) -> PseudoInertia :
            Returns the Pseudo Inertia from an Inertia object.
        """
    @staticmethod
    def FromMatrix(pseudo_inertia_matrix: numpy.ndarray) -> PseudoInertia: 
        """
        FromMatrix( (numpy.ndarray)pseudo_inertia_matrix) -> PseudoInertia :
            Returns the Pseudo Inertia from a 4x4 matrix.
        """
    @staticmethod
    @typing.overload
    def __array__(arg1: PseudoInertia) -> numpy.ndarray: 
        """
        __array__( (PseudoInertia)arg1) -> numpy.ndarray
        """
    @typing.overload
    def __array__(self, dtype: object = None, copy: object = None) -> numpy.ndarray: ...
    def __copy__(self) -> PseudoInertia: 
        """
        __copy__( (PseudoInertia)self) -> PseudoInertia :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> PseudoInertia: 
        """
        __deepcopy__( (PseudoInertia)self, (dict)memo) -> PseudoInertia :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __getinitargs__(arg1: PseudoInertia) -> tuple: 
        """
        __getinitargs__( (PseudoInertia)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, clone: PseudoInertia) -> object: 
        """
        __init__( (object)self, (float)mass, (numpy.ndarray)h, (numpy.ndarray)sigma) -> None :
            Initialize from mass, vector part of the pseudo inertia and matrix part of the pseudo inertia.
        """
    @typing.overload
    def __init__(self, clone: PseudoInertia) -> None: ...
    @typing.overload
    def __init__(self, mass: float, h: numpy.ndarray, sigma: numpy.ndarray) -> None: ...
    @staticmethod
    def __repr__(arg1: PseudoInertia) -> object: 
        """
        __repr__( (PseudoInertia)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: PseudoInertia) -> object: 
        """
        __str__( (PseudoInertia)arg1) -> object
        """
    @staticmethod
    def cast(arg1: PseudoInertia) -> PseudoInertia: 
        """
        cast( (PseudoInertia)arg1) -> PseudoInertia :
            Returns a cast of *this.
        """
    def copy(self) -> PseudoInertia: 
        """
        copy( (PseudoInertia)self) -> PseudoInertia :
            Returns a copy of *this.
        """
    def toDynamicParameters(self) -> numpy.ndarray: 
        """
        toDynamicParameters( (PseudoInertia)self) -> numpy.ndarray :
            Returns the dynamic parameters representation.
        """
    def toInertia(self) -> Inertia: 
        """
        toInertia( (PseudoInertia)self) -> Inertia :
            Returns the inertia representation.
        """
    def toMatrix(self) -> numpy.ndarray: 
        """
        toMatrix( (PseudoInertia)self) -> numpy.ndarray :
            Returns the pseudo inertia as a 4x4 matrix.
        """
    @property
    def h(self) -> numpy.ndarray:
        """
        Vector part of the Pseudo Inertia.

        :type: numpy.ndarray
        """
    @property
    def mass(self) -> float:
        """
        Mass of the Pseudo Inertia.

        :type: float
        """
    @property
    def sigma(self) -> numpy.ndarray:
        """
        Matrix part of the Pseudo Inertia.

        :type: numpy.ndarray
        """
    __safe_for_unpickling__ = True
    pass

class Quaternion(Boost.Python.instance):
    """
    Quaternion representing rotation.

    Supported operations ('q is a Quaternion, 'v' is a Vector3): 'q*q' (rotation composition), 'q*=q', 'q*v' (rotating 'v' by 'q'), 'q==q', 'q!=q', 'q[0..3]'.
    """
    @staticmethod
    def FromTwoVectors(a: numpy.ndarray, b: numpy.ndarray) -> Quaternion: 
        """
        FromTwoVectors( (numpy.ndarray)a, (numpy.ndarray)b) -> Quaternion :
            Returns the quaternion which transforms a into b through a rotation.
        """
    @staticmethod
    def Identity() -> Quaternion: 
        """
        Identity() -> Quaternion :
            Returns a quaternion representing an identity rotation.
        """
    @staticmethod
    def __abs__(arg1: Quaternion) -> float: 
        """
        __abs__( (Quaternion)arg1) -> float
        """
    @staticmethod
    def __eq__(arg1: Quaternion, arg2: Quaternion) -> bool: 
        """
        __eq__( (Quaternion)arg1, (Quaternion)arg2) -> bool
        """
    @staticmethod
    def __getitem__(arg1: Quaternion, arg2: int) -> float: 
        """
        __getitem__( (Quaternion)arg1, (int)arg2) -> float
        """
    @staticmethod
    def __imul__(arg1: object, arg2: Quaternion) -> object: 
        """
        __imul__( (object)arg1, (Quaternion)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> object: 
        """
        __init__( (object)arg1, (numpy.ndarray)R) -> object :
            Initialize from rotation matrix.
            	R : a rotation matrix 3x3.
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, R: numpy.ndarray) -> object: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, aa: AngleAxis) -> object: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, quat: Quaternion) -> object: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, u: numpy.ndarray, v: numpy.ndarray) -> object: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, vec4: numpy.ndarray) -> object: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, w: float, x: float, y: float, z: float) -> object: ...
    @staticmethod
    def __len__() -> int: 
        """
        __len__() -> int
        """
    @staticmethod
    @typing.overload
    def __mul__(arg1: Quaternion, arg2: Quaternion) -> object: 
        """
        __mul__( (Quaternion)arg1, (Quaternion)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __mul__(arg1: Quaternion, arg2: numpy.ndarray) -> object: ...
    @staticmethod
    def __ne__(arg1: Quaternion, arg2: Quaternion) -> bool: 
        """
        __ne__( (Quaternion)arg1, (Quaternion)arg2) -> bool
        """
    @staticmethod
    def __repr__(arg1: Quaternion) -> str: 
        """
        __repr__( (Quaternion)arg1) -> str
        """
    @staticmethod
    def __setitem__(arg1: Quaternion, arg2: int, arg3: float) -> None: 
        """
        __setitem__( (Quaternion)arg1, (int)arg2, (float)arg3) -> None
        """
    @staticmethod
    def __str__(arg1: Quaternion) -> str: 
        """
        __str__( (Quaternion)arg1) -> str
        """
    def _transformVector(self, vector: numpy.ndarray) -> numpy.ndarray: 
        """
        _transformVector( (Quaternion)self, (numpy.ndarray)vector) -> numpy.ndarray :
            Rotation of a vector by a quaternion.
        """
    @staticmethod
    def angularDistance(arg1: Quaternion, arg2: Quaternion) -> float: 
        """
        angularDistance( (Quaternion)arg1, (Quaternion)arg2) -> float :
            Returns the angle (in radian) between two rotations.
        """
    @typing.overload
    def assign(self, aa: AngleAxis) -> Quaternion: 
        """
        assign( (Quaternion)self, (Quaternion)quat) -> Quaternion :
            Set *this from an quaternion quat and returns a reference to *this.
        """
    @typing.overload
    def assign(self, quat: Quaternion) -> Quaternion: ...
    def coeffs(self) -> object: 
        """
        coeffs( (Quaternion)self) -> object :
            Returns a vector of the coefficients (x,y,z,w)
        """
    def conjugate(self) -> Quaternion: 
        """
        conjugate( (Quaternion)self) -> Quaternion :
            Returns the conjugated quaternion.
            The conjugate of a quaternion represents the opposite rotation.
        """
    def dot(self, other: Quaternion) -> float: 
        """
        dot( (Quaternion)self, (Quaternion)other) -> float :
            Returns the dot product of *this with an other Quaternion.
            Geometrically speaking, the dot product of two unit quaternions corresponds to the cosine of half the angle between the two rotations.
        """
    def id(self) -> int: 
        """
        id( (Quaternion)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def inverse(self) -> Quaternion: 
        """
        inverse( (Quaternion)self) -> Quaternion :
            Returns the quaternion describing the inverse rotation.
        """
    def isApprox(self, other_: Quaternion, prec: float) -> bool: 
        """
        isApprox( (Quaternion)self, (Quaternion)other [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to other, within the precision determined by prec.
        """
    def matrix(self) -> numpy.ndarray: 
        """
        matrix( (Quaternion)self) -> numpy.ndarray :
            Returns an equivalent 3x3 rotation matrix. Similar to toRotationMatrix.
        """
    def norm(self) -> float: 
        """
        norm( (Quaternion)self) -> float :
            Returns the norm of the quaternion's coefficients.
        """
    def normalize(self) -> Quaternion: 
        """
        normalize( (Quaternion)self) -> Quaternion :
            Normalizes the quaternion *this.
        """
    def normalized(self) -> Quaternion: 
        """
        normalized( (Quaternion)self) -> Quaternion :
            Returns a normalized copy of *this.
        """
    def setFromTwoVectors(self, a: numpy.ndarray, b: numpy.ndarray) -> Quaternion: 
        """
        setFromTwoVectors( (Quaternion)self, (numpy.ndarray)a, (numpy.ndarray)b) -> Quaternion :
            Set *this to be the quaternion which transforms a into b through a rotation.
        """
    def setIdentity(self) -> Quaternion: 
        """
        setIdentity( (Quaternion)self) -> Quaternion :
            Set *this to the identity rotation.
        """
    def slerp(self, t: float, other: Quaternion) -> Quaternion: 
        """
        slerp( (Quaternion)self, (float)t, (Quaternion)other) -> Quaternion :
            Returns the spherical linear interpolation between the two quaternions *this and other at the parameter t in [0;1].
        """
    def squaredNorm(self) -> float: 
        """
        squaredNorm( (Quaternion)self) -> float :
            Returns the squared norm of the quaternion's coefficients.
        """
    @staticmethod
    def toRotationMatrix(arg1: Quaternion) -> numpy.ndarray: 
        """
        toRotationMatrix( (Quaternion)arg1) -> numpy.ndarray :
            Returns an equivalent rotation matrix.
        """
    def vec(self) -> numpy.ndarray: 
        """
        vec( (Quaternion)self) -> numpy.ndarray :
            Returns a vector expression of the imaginary part (x,y,z).
        """
    @property
    def w(self) -> float:
        """
        The w coefficient.

        :type: float
        """
    @property
    def x(self) -> float:
        """
        The x coefficient.

        :type: float
        """
    @property
    def y(self) -> float:
        """
        The y coefficient.

        :type: float
        """
    @property
    def z(self) -> float:
        """
        The z coefficient.

        :type: float
        """
    pass

class RigidConstraintData(Boost.Python.instance):
    """
    Rigid constraint data associated to a RigidConstraintModel for contact dynamic algorithms.
    """
    @staticmethod
    def __eq__(arg1: RigidConstraintData, arg2: RigidConstraintData) -> object: 
        """
        __eq__( (RigidConstraintData)arg1, (RigidConstraintData)arg2) -> object
        """
    def __init__(self, contact_model: RigidConstraintModel) -> None: 
        """
        __init__( (object)self, (RigidConstraintModel)contact_model) -> None :
            Default constructor.
        """
    @staticmethod
    def __ne__(arg1: RigidConstraintData, arg2: RigidConstraintData) -> object: 
        """
        __ne__( (RigidConstraintData)arg1, (RigidConstraintData)arg2) -> object
        """
    @property
    def c1Mc2(self) -> SE3:
        """
        Relative displacement between the two frames.

        :type: SE3
        """
    @property
    def contact1_acceleration_drift(self) -> Motion:
        """
        Current contact drift acceleration (acceleration only due to the Coriolis and centrifugal effects) of the contact frame 1.

        :type: Motion
        """
    @property
    def contact1_velocity(self) -> Motion:
        """
        Current contact Spatial velocity of the constraint 1.

        :type: Motion
        """
    @property
    def contact2_acceleration_drift(self) -> Motion:
        """
        Current contact drift acceleration (acceleration only due to the Coriolis and centrifugal effects) of the contact frame 2.

        :type: Motion
        """
    @property
    def contact2_velocity(self) -> Motion:
        """
        Current contact Spatial velocity of the constraint 2.

        :type: Motion
        """
    @property
    def contact_acceleration(self) -> Motion:
        """
        Current contact Spatial acceleration.

        :type: Motion
        """
    @property
    def contact_acceleration_desired(self) -> Motion:
        """
        Desired contact acceleration.

        :type: Motion
        """
    @property
    def contact_acceleration_deviation(self) -> Motion:
        """
        Contact deviation from the reference acceleration (a.k.a the error).

        :type: Motion
        """
    @property
    def contact_acceleration_error(self) -> Motion:
        """
        Current contact spatial error (due to the integration step).

        :type: Motion
        """
    @property
    def contact_force(self) -> Force:
        """
        Constraint force.

        :type: Force
        """
    @property
    def contact_placement_error(self) -> Motion:
        """
        Current contact placement error between the two contact Frames.
        This corresponds to the relative placement between the two contact Frames seen as a Motion error.

        :type: Motion
        """
    @property
    def contact_velocity_error(self) -> Motion:
        """
        Current contact Spatial velocity error between the two contact Frames.
        This corresponds to the relative velocity between the two contact Frames.

        :type: Motion
        """
    @property
    def extended_motion_propagators_joint1(self) -> Sequence[FloatMat6]:
        """
        Extended force/motion propagators for joint 1.

        :type: StdVec_Matrix6
        """
    @property
    def extended_motion_propagators_joint2(self) -> Sequence[FloatMat6]:
        """
        Extended force/motion propagators for joint 2.

        :type: StdVec_Matrix6
        """
    @property
    def lambdas_joint1(self) -> Sequence[FloatMat6]:
        """
        Extended force/motion propagators for joint 1.

        :type: StdVec_Matrix6
        """
    @property
    def oMc1(self) -> SE3:
        """
        Placement of the constraint frame 1 with respect to the WORLD frame.

        :type: SE3
        """
    @property
    def oMc2(self) -> SE3:
        """
        Placement of the constraint frame 2 with respect to the WORLD frame.

        :type: SE3
        """
    pass

class RigidConstraintModel(Boost.Python.instance):
    """
    Rigid contact model for contact dynamic algorithms.
    """
    @staticmethod
    def __eq__(arg1: RigidConstraintModel, arg2: RigidConstraintModel) -> object: 
        """
        __eq__( (RigidConstraintModel)arg1, (RigidConstraintModel)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, clone: RigidConstraintModel) -> object: 
        """
        __init__( (object)self, (ContactType)contact_type, (Model)model, (int)joint1_id, (SE3)joint1_placement, (int)joint2_id, (SE3)joint2_placement [, (ReferenceFrame)reference_frame]) -> None :
            Contructor from a given ContactType, joint index and placement for the two joints implied in the constraint.
        """
    @typing.overload
    def __init__(self, contact_type: ContactType, model: Model, joint1_id: int, joint1_placement: SE3, joint2_id: int, joint2_placement_: SE3, reference_frame: ReferenceFrame) -> None: ...
    @typing.overload
    def __init__(self, contact_type: ContactType, model: Model, joint1_id: int, joint1_placement_: SE3, reference_frame: ReferenceFrame) -> None: ...
    @typing.overload
    def __init__(self, contact_type: ContactType, model: Model, joint1_id_: int, reference_frame: ReferenceFrame) -> None: ...
    @staticmethod
    def __ne__(arg1: RigidConstraintModel, arg2: RigidConstraintModel) -> object: 
        """
        __ne__( (RigidConstraintModel)arg1, (RigidConstraintModel)arg2) -> object
        """
    @staticmethod
    def cast(arg1: RigidConstraintModel) -> RigidConstraintModel: 
        """
        cast( (RigidConstraintModel)arg1) -> RigidConstraintModel :
            Returns a cast of *this.
        """
    @staticmethod
    def createData(arg1: RigidConstraintModel) -> RigidConstraintData: 
        """
        createData( (RigidConstraintModel)arg1) -> RigidConstraintData :
            Create a Data object for the given model.
        """
    @staticmethod
    def size(arg1: RigidConstraintModel) -> int: 
        """
        size( (RigidConstraintModel)arg1) -> int :
            Size of the constraint
        """
    @property
    def colwise_joint1_sparsity(self) -> numpy.ndarray:
        """
        Sparsity pattern associated to joint 1.

        :type: numpy.ndarray
        """
    @property
    def colwise_joint2_sparsity(self) -> numpy.ndarray:
        """
        Sparsity pattern associated to joint 2.

        :type: numpy.ndarray
        """
    @property
    def colwise_span_indexes(self) -> Sequence[int]:
        """
        Indexes of the columns spanned by the constraints.

        :type: StdVec_Index
        """
    @property
    def corrector(self) -> BaumgarteCorrectorParameters:
        """
        Corrector parameters.

        :type: BaumgarteCorrectorParameters
        """
    @property
    def desired_contact_acceleration(self) -> Motion:
        """
        Desired contact spatial acceleration.

        :type: Motion
        """
    @property
    def desired_contact_placement(self) -> SE3:
        """
        Desired contact placement.

        :type: SE3
        """
    @property
    def desired_contact_velocity(self) -> Motion:
        """
        Desired contact spatial velocity.

        :type: Motion
        """
    @property
    def joint1_id(self) -> int:
        """
        Index of first parent joint in the model tree.

        :type: int
        """
    @property
    def joint1_placement(self) -> SE3:
        """
        Relative placement with respect to the frame of joint1.

        :type: SE3
        """
    @property
    def joint2_id(self) -> int:
        """
        Index of second parent joint in the model tree.

        :type: int
        """
    @property
    def joint2_placement(self) -> SE3:
        """
        Relative placement with respect to the frame of joint2.

        :type: SE3
        """
    @property
    def name(self) -> str:
        """
        Name of the contact.

        :type: str
        """
    @property
    def reference_frame(self) -> ReferenceFrame:
        """
        Reference frame where the constraint is expressed (WORLD, LOCAL_WORLD_ALIGNED or LOCAL).

        :type: ReferenceFrame
        """
    @property
    def type(self) -> ContactType:
        """
        Type of the contact.

        :type: ContactType
        """
    pass

class SE3(Boost.Python.instance):
    """
    SE3 transformation defined by a 3d vector and a rotation matrix.
    """
    @staticmethod
    def Identity() -> SE3: 
        """
        Identity() -> SE3 :
            Returns the identity transformation.
        """
    @staticmethod
    def Interpolate(A: SE3, B: SE3, alpha: float) -> SE3: 
        """
        Interpolate( (SE3)A, (SE3)B, (float)alpha) -> SE3 :
            Linear interpolation on the SE3 manifold.
            
            This method computes the linear interpolation between A and B, such that the result C = A + (B-A)*t if it would be applied on classic Euclidian space.
            This operation is very similar to the SLERP operation on Rotations.
            Parameters:
            	A: Initial transformation
            	B: Target transformation
            	alpha: Interpolation factor
        """
    @staticmethod
    def Random() -> SE3: 
        """
        Random() -> SE3 :
            Returns a random transformation.
        """
    @staticmethod
    @typing.overload
    def __array__(arg1: SE3) -> numpy.ndarray: 
        """
        __array__( (SE3)arg1) -> numpy.ndarray
        """
    @typing.overload
    def __array__(self, dtype: object = None, copy: object = None) -> numpy.ndarray: ...
    def __copy__(self) -> SE3: 
        """
        __copy__( (SE3)self) -> SE3 :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> SE3: 
        """
        __deepcopy__( (SE3)self, (dict)memo) -> SE3 :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: SE3, arg2: SE3) -> object: 
        """
        __eq__( (SE3)arg1, (SE3)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(arg1: SE3) -> tuple: 
        """
        __getinitargs__( (SE3)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, clone: SE3) -> object: 
        """
        __init__( (object)self) -> None :
            Default constructor.
        """
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None: ...
    @typing.overload
    def __init__(self, clone: SE3) -> None: ...
    @typing.overload
    def __init__(self, int: int) -> None: ...
    @typing.overload
    def __init__(self, quat: Quaternion, translation: numpy.ndarray) -> None: ...
    @typing.overload
    def __init__(self, rotation: numpy.ndarray, translation: numpy.ndarray) -> None: ...
    @staticmethod
    def __invert__(arg1: SE3) -> SE3: 
        """
        __invert__( (SE3)arg1) -> SE3 :
            Returns the inverse of *this.
        """
    @staticmethod
    @typing.overload
    def __mul__(arg1: SE3, arg2: Force) -> Force: 
        """
        __mul__( (SE3)arg1, (SE3)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __mul__(arg1: SE3, arg2: Inertia) -> Inertia: ...
    @staticmethod
    @typing.overload
    def __mul__(arg1: SE3, arg2: Motion) -> Motion: ...
    @staticmethod
    @typing.overload
    def __mul__(arg1: SE3, arg2: SE3) -> object: ...
    @staticmethod
    @typing.overload
    def __mul__(arg1: SE3, arg2: numpy.ndarray) -> numpy.ndarray: ...
    @staticmethod
    def __ne__(arg1: SE3, arg2: SE3) -> object: 
        """
        __ne__( (SE3)arg1, (SE3)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: SE3) -> str: 
        """
        __repr__( (SE3)arg1) -> str
        """
    @staticmethod
    def __str__(arg1: SE3) -> object: 
        """
        __str__( (SE3)arg1) -> object
        """
    @typing.overload
    def act(self, M: SE3) -> SE3: 
        """
        act( (SE3)self, (numpy.ndarray)point) -> numpy.ndarray :
            Returns a point which is the result of the entry point transforms by *this.
        """
    @typing.overload
    def act(self, force: Force) -> Force: ...
    @typing.overload
    def act(self, inertia: Inertia) -> Inertia: ...
    @typing.overload
    def act(self, motion: Motion) -> Motion: ...
    @typing.overload
    def act(self, point: numpy.ndarray) -> numpy.ndarray: ...
    @typing.overload
    def actInv(self, M: SE3) -> SE3: 
        """
        actInv( (SE3)self, (numpy.ndarray)point) -> numpy.ndarray :
            Returns a point which is the result of the entry point by the inverse of *this.
        """
    @typing.overload
    def actInv(self, force: Force) -> Force: ...
    @typing.overload
    def actInv(self, inertia: Inertia) -> Inertia: ...
    @typing.overload
    def actInv(self, motion: Motion) -> Motion: ...
    @typing.overload
    def actInv(self, point: numpy.ndarray) -> numpy.ndarray: ...
    @staticmethod
    def cast(arg1: SE3) -> SE3: 
        """
        cast( (SE3)arg1) -> SE3 :
            Returns a cast of *this.
        """
    def copy(self) -> SE3: 
        """
        copy( (SE3)self) -> SE3 :
            Returns a copy of *this.
        """
    def inverse(self) -> SE3: 
        """
        inverse( (SE3)self) -> SE3 :
            Returns the inverse transform
        """
    def isApprox(self, other_: SE3, prec: float = 1e-12) -> bool: 
        """
        isApprox( (SE3)self, (SE3)other [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to other, within the precision given by prec.
        """
    def isIdentity(self, prec: float = 1e-12) -> bool: 
        """
        isIdentity( (SE3)self [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to the identity placement, within the precision given by prec.
        """
    def setIdentity(self) -> None: 
        """
        setIdentity( (SE3)self) -> None :
            Set *this to the identity placement.
        """
    def setRandom(self) -> None: 
        """
        setRandom( (SE3)self) -> None :
            Set *this to a random placement.
        """
    def toActionMatrix(self) -> numpy.ndarray: 
        """
        toActionMatrix( (SE3)self) -> numpy.ndarray :
            Returns the related action matrix (acting on Motion).
        """
    def toActionMatrixInverse(self) -> numpy.ndarray: 
        """
        toActionMatrixInverse( (SE3)self) -> numpy.ndarray :
            Returns the inverse of the action matrix (acting on Motion).
            This is equivalent to do m.inverse().toActionMatrix()
        """
    def toDualActionMatrix(self) -> numpy.ndarray: 
        """
        toDualActionMatrix( (SE3)self) -> numpy.ndarray :
            Returns the related dual action matrix (acting on Force).
        """
    @property
    def action(self) -> numpy.ndarray:
        """
        Returns the related action matrix (acting on Motion).

        :type: numpy.ndarray
        """
    @property
    def actionInverse(self) -> numpy.ndarray:
        """
        Returns the inverse of the action matrix (acting on Motion).
        This is equivalent to do m.inverse().action

        :type: numpy.ndarray
        """
    @property
    def dualAction(self) -> numpy.ndarray:
        """
        Returns the related dual action matrix (acting on Force).

        :type: numpy.ndarray
        """
    @property
    def homogeneous(self) -> numpy.ndarray:
        """
        Returns the equivalent homegeneous matrix (acting on SE3).

        :type: numpy.ndarray
        """
    @property
    def np(self) -> numpy.ndarray:
        """
        :type: numpy.ndarray
        """
    @property
    def rotation(self) -> numpy.ndarray:
        """
        The rotation part of the transformation.

        :type: numpy.ndarray
        """
    @property
    def translation(self) -> numpy.ndarray:
        """
        The translation part of the transformation.

        :type: numpy.ndarray
        """
    __instance_size__ = 120
    __safe_for_unpickling__ = True
    pass

class SolverStats(Boost.Python.instance):
    def __init__(self, max_it: int) -> None: 
        """
        __init__( (object)self, (int)max_it) -> None :
            Default constructor
        """
    def reset(self) -> None: 
        """
        reset( (SolverStats)self) -> None :
            Reset the stasts.
        """
    def size(self) -> int: 
        """
        size( (SolverStats)self) -> int :
            Size of the vectors stored in the structure.
        """
    @property
    def cholesky_update_count(self) -> int:
        """
        Number of Cholesky updates performed by the algorithm.

        :type: int
        """
    @property
    def complementarity(self) -> Sequence[float]:
        """
        :type: StdVec_Scalar
        """
    @property
    def dual_feasibility(self) -> Sequence[float]:
        """
        :type: StdVec_Scalar
        """
    @property
    def dual_feasibility_ncp(self) -> Sequence[float]:
        """
        :type: StdVec_Scalar
        """
    @property
    def it(self) -> int:
        """
        Number of iterations performed by the algorithm.

        :type: int
        """
    @property
    def primal_feasibility(self) -> Sequence[float]:
        """
        :type: StdVec_Scalar
        """
    @property
    def rho(self) -> Sequence[float]:
        """
        :type: StdVec_Scalar
        """
    __instance_size__ = 152
    pass

class StdMap_String_VectorXd(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdMap_String_VectorXd, arg2: object) -> bool: 
        """
        __contains__( (StdMap_String_VectorXd)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(arg1: StdMap_String_VectorXd, arg2: object) -> None: 
        """
        __delitem__( (StdMap_String_VectorXd)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdMap_String_VectorXd) -> tuple: 
        """
        __getinitargs__( (StdMap_String_VectorXd)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdMap_String_VectorXd) -> int: 
        """
        __len__( (StdMap_String_VectorXd)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdMap_String_VectorXd, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdMap_String_VectorXd)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 72
    __safe_for_unpickling__ = True
    pass

class StdVec_Bool(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Bool, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Bool)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_Bool: 
        """
        __copy__( (StdVec_Bool)self) -> StdVec_Bool :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_Bool: 
        """
        __deepcopy__( (StdVec_Bool)self, (dict)memo) -> StdVec_Bool :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Bool, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Bool)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_Bool) -> tuple: 
        """
        __getinitargs__( (StdVec_Bool)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_Bool) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: bool) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_Bool) -> int: 
        """
        __len__( (StdVec_Bool)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Bool, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Bool)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_Bool, arg2: object) -> None: 
        """
        append( (StdVec_Bool)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_Bool: 
        """
        copy( (StdVec_Bool)self) -> StdVec_Bool :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_Bool, arg2: object) -> None: 
        """
        extend( (StdVec_Bool)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_Bool)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_Bool)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_Bool)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 72
    __safe_for_unpickling__ = True
    pass

class StdVec_CollisionPair(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_CollisionPair, arg2: object) -> bool: 
        """
        __contains__( (StdVec_CollisionPair)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_CollisionPair: 
        """
        __copy__( (StdVec_CollisionPair)self) -> StdVec_CollisionPair :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_CollisionPair: 
        """
        __deepcopy__( (StdVec_CollisionPair)self, (dict)memo) -> StdVec_CollisionPair :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_CollisionPair, arg2: object) -> None: 
        """
        __delitem__( (StdVec_CollisionPair)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_CollisionPair) -> tuple: 
        """
        __getinitargs__( (StdVec_CollisionPair)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_CollisionPair) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: CollisionPair) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_CollisionPair) -> int: 
        """
        __len__( (StdVec_CollisionPair)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_CollisionPair, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_CollisionPair)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_CollisionPair, arg2: object) -> None: 
        """
        append( (StdVec_CollisionPair)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_CollisionPair: 
        """
        copy( (StdVec_CollisionPair)self) -> StdVec_CollisionPair :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_CollisionPair, arg2: object) -> None: 
        """
        extend( (StdVec_CollisionPair)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_CollisionPair)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_CollisionPair)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_CollisionPair)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_CoulombFrictionCone(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_CoulombFrictionCone, arg2: object) -> bool: 
        """
        __contains__( (StdVec_CoulombFrictionCone)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_CoulombFrictionCone: 
        """
        __copy__( (StdVec_CoulombFrictionCone)self) -> StdVec_CoulombFrictionCone :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_CoulombFrictionCone: 
        """
        __deepcopy__( (StdVec_CoulombFrictionCone)self, (dict)memo) -> StdVec_CoulombFrictionCone :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_CoulombFrictionCone, arg2: object) -> None: 
        """
        __delitem__( (StdVec_CoulombFrictionCone)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_CoulombFrictionCone) -> tuple: 
        """
        __getinitargs__( (StdVec_CoulombFrictionCone)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_CoulombFrictionCone) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: CoulombFrictionCone) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_CoulombFrictionCone) -> int: 
        """
        __len__( (StdVec_CoulombFrictionCone)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_CoulombFrictionCone, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_CoulombFrictionCone)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_CoulombFrictionCone, arg2: object) -> None: 
        """
        append( (StdVec_CoulombFrictionCone)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_CoulombFrictionCone: 
        """
        copy( (StdVec_CoulombFrictionCone)self) -> StdVec_CoulombFrictionCone :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_CoulombFrictionCone, arg2: object) -> None: 
        """
        extend( (StdVec_CoulombFrictionCone)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_CoulombFrictionCone)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_CoulombFrictionCone)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_CoulombFrictionCone)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_Scalar(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Scalar, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Scalar)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_Scalar: 
        """
        __copy__( (StdVec_Scalar)self) -> StdVec_Scalar :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_Scalar: 
        """
        __deepcopy__( (StdVec_Scalar)self, (dict)memo) -> StdVec_Scalar :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Scalar, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Scalar)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_Scalar) -> tuple: 
        """
        __getinitargs__( (StdVec_Scalar)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_Scalar) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: float) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_Scalar) -> int: 
        """
        __len__( (StdVec_Scalar)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Scalar, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Scalar)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_Scalar, arg2: object) -> None: 
        """
        append( (StdVec_Scalar)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_Scalar: 
        """
        copy( (StdVec_Scalar)self) -> StdVec_Scalar :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_Scalar, arg2: object) -> None: 
        """
        extend( (StdVec_Scalar)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_Scalar)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_Scalar)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_Scalar)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_DualCoulombFrictionCone(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_DualCoulombFrictionCone, arg2: object) -> bool: 
        """
        __contains__( (StdVec_DualCoulombFrictionCone)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_DualCoulombFrictionCone: 
        """
        __copy__( (StdVec_DualCoulombFrictionCone)self) -> StdVec_DualCoulombFrictionCone :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_DualCoulombFrictionCone: 
        """
        __deepcopy__( (StdVec_DualCoulombFrictionCone)self, (dict)memo) -> StdVec_DualCoulombFrictionCone :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_DualCoulombFrictionCone, arg2: object) -> None: 
        """
        __delitem__( (StdVec_DualCoulombFrictionCone)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_DualCoulombFrictionCone) -> tuple: 
        """
        __getinitargs__( (StdVec_DualCoulombFrictionCone)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_DualCoulombFrictionCone) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: DualCoulombFrictionCone) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_DualCoulombFrictionCone) -> int: 
        """
        __len__( (StdVec_DualCoulombFrictionCone)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_DualCoulombFrictionCone, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_DualCoulombFrictionCone)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_DualCoulombFrictionCone, arg2: object) -> None: 
        """
        append( (StdVec_DualCoulombFrictionCone)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_DualCoulombFrictionCone: 
        """
        copy( (StdVec_DualCoulombFrictionCone)self) -> StdVec_DualCoulombFrictionCone :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_DualCoulombFrictionCone, arg2: object) -> None: 
        """
        extend( (StdVec_DualCoulombFrictionCone)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_DualCoulombFrictionCone)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_DualCoulombFrictionCone)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_DualCoulombFrictionCone)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_Force(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Force, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Force)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_Force: 
        """
        __copy__( (StdVec_Force)self) -> StdVec_Force :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_Force: 
        """
        __deepcopy__( (StdVec_Force)self, (dict)memo) -> StdVec_Force :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Force, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Force)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_Force) -> tuple: 
        """
        __getinitargs__( (StdVec_Force)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_Force) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: Force) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_Force) -> int: 
        """
        __len__( (StdVec_Force)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Force, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Force)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_Force, arg2: object) -> None: 
        """
        append( (StdVec_Force)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_Force: 
        """
        copy( (StdVec_Force)self) -> StdVec_Force :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_Force, arg2: object) -> None: 
        """
        extend( (StdVec_Force)arg1, (object)arg2) -> None
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_Force)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_Frame(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Frame, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Frame)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_Frame: 
        """
        __copy__( (StdVec_Frame)self) -> StdVec_Frame :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_Frame: 
        """
        __deepcopy__( (StdVec_Frame)self, (dict)memo) -> StdVec_Frame :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Frame, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Frame)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_Frame) -> tuple: 
        """
        __getinitargs__( (StdVec_Frame)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_Frame) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: Frame) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_Frame) -> int: 
        """
        __len__( (StdVec_Frame)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Frame, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Frame)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_Frame, arg2: object) -> None: 
        """
        append( (StdVec_Frame)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_Frame: 
        """
        copy( (StdVec_Frame)self) -> StdVec_Frame :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_Frame, arg2: object) -> None: 
        """
        extend( (StdVec_Frame)arg1, (object)arg2) -> None
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_Frame)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_GeometryModel(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_GeometryModel, arg2: object) -> bool: 
        """
        __contains__( (StdVec_GeometryModel)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_GeometryModel: 
        """
        __copy__( (StdVec_GeometryModel)self) -> StdVec_GeometryModel :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_GeometryModel: 
        """
        __deepcopy__( (StdVec_GeometryModel)self, (dict)memo) -> StdVec_GeometryModel :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_GeometryModel, arg2: object) -> None: 
        """
        __delitem__( (StdVec_GeometryModel)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_GeometryModel) -> tuple: 
        """
        __getinitargs__( (StdVec_GeometryModel)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_GeometryModel) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: GeometryModel) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_GeometryModel) -> int: 
        """
        __len__( (StdVec_GeometryModel)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_GeometryModel, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_GeometryModel)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_GeometryModel, arg2: object) -> None: 
        """
        append( (StdVec_GeometryModel)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_GeometryModel: 
        """
        copy( (StdVec_GeometryModel)self) -> StdVec_GeometryModel :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_GeometryModel, arg2: object) -> None: 
        """
        extend( (StdVec_GeometryModel)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_GeometryModel)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_GeometryModel)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_GeometryModel)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_GeometryObject(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_GeometryObject, arg2: object) -> bool: 
        """
        __contains__( (StdVec_GeometryObject)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_GeometryObject: 
        """
        __copy__( (StdVec_GeometryObject)self) -> StdVec_GeometryObject :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_GeometryObject: 
        """
        __deepcopy__( (StdVec_GeometryObject)self, (dict)memo) -> StdVec_GeometryObject :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_GeometryObject, arg2: object) -> None: 
        """
        __delitem__( (StdVec_GeometryObject)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_GeometryObject) -> tuple: 
        """
        __getinitargs__( (StdVec_GeometryObject)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_GeometryObject) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: GeometryObject) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_GeometryObject) -> int: 
        """
        __len__( (StdVec_GeometryObject)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_GeometryObject, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_GeometryObject)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_GeometryObject, arg2: object) -> None: 
        """
        append( (StdVec_GeometryObject)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_GeometryObject: 
        """
        copy( (StdVec_GeometryObject)self) -> StdVec_GeometryObject :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_GeometryObject, arg2: object) -> None: 
        """
        extend( (StdVec_GeometryObject)arg1, (object)arg2) -> None
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_GeometryObject)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_Index(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Index, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Index)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_Index: 
        """
        __copy__( (StdVec_Index)self) -> StdVec_Index :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_Index: 
        """
        __deepcopy__( (StdVec_Index)self, (dict)memo) -> StdVec_Index :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Index, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Index)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_Index) -> tuple: 
        """
        __getinitargs__( (StdVec_Index)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_Index) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: int) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_Index) -> int: 
        """
        __len__( (StdVec_Index)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Index, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Index)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_Index, arg2: object) -> None: 
        """
        append( (StdVec_Index)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_Index: 
        """
        copy( (StdVec_Index)self) -> StdVec_Index :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_Index, arg2: object) -> None: 
        """
        extend( (StdVec_Index)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_Index)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_Index)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_Index)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_IndexVector(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_IndexVector, arg2: object) -> bool: 
        """
        __contains__( (StdVec_IndexVector)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_IndexVector: 
        """
        __copy__( (StdVec_IndexVector)self) -> StdVec_IndexVector :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_IndexVector: 
        """
        __deepcopy__( (StdVec_IndexVector)self, (dict)memo) -> StdVec_IndexVector :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_IndexVector, arg2: object) -> None: 
        """
        __delitem__( (StdVec_IndexVector)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_IndexVector) -> tuple: 
        """
        __getinitargs__( (StdVec_IndexVector)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_IndexVector) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: StdVec_Index) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_IndexVector) -> int: 
        """
        __len__( (StdVec_IndexVector)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_IndexVector, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_IndexVector)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_IndexVector, arg2: object) -> None: 
        """
        append( (StdVec_IndexVector)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_IndexVector: 
        """
        copy( (StdVec_IndexVector)self) -> StdVec_IndexVector :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_IndexVector, arg2: object) -> None: 
        """
        extend( (StdVec_IndexVector)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_IndexVector)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_IndexVector)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_IndexVector)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_Inertia(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Inertia, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Inertia)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_Inertia: 
        """
        __copy__( (StdVec_Inertia)self) -> StdVec_Inertia :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_Inertia: 
        """
        __deepcopy__( (StdVec_Inertia)self, (dict)memo) -> StdVec_Inertia :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Inertia, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Inertia)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_Inertia) -> tuple: 
        """
        __getinitargs__( (StdVec_Inertia)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_Inertia) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: Inertia) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_Inertia) -> int: 
        """
        __len__( (StdVec_Inertia)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Inertia, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Inertia)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_Inertia, arg2: object) -> None: 
        """
        append( (StdVec_Inertia)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_Inertia: 
        """
        copy( (StdVec_Inertia)self) -> StdVec_Inertia :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_Inertia, arg2: object) -> None: 
        """
        extend( (StdVec_Inertia)arg1, (object)arg2) -> None
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_Inertia)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_JointDataVector(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_JointDataVector, arg2: object) -> bool: 
        """
        __contains__( (StdVec_JointDataVector)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_JointDataVector: 
        """
        __copy__( (StdVec_JointDataVector)self) -> StdVec_JointDataVector :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_JointDataVector: 
        """
        __deepcopy__( (StdVec_JointDataVector)self, (dict)memo) -> StdVec_JointDataVector :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_JointDataVector, arg2: object) -> None: 
        """
        __delitem__( (StdVec_JointDataVector)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_JointDataVector) -> tuple: 
        """
        __getinitargs__( (StdVec_JointDataVector)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_JointDataVector) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: JointData) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_JointDataVector) -> int: 
        """
        __len__( (StdVec_JointDataVector)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_JointDataVector, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_JointDataVector)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_JointDataVector, arg2: object) -> None: 
        """
        append( (StdVec_JointDataVector)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_JointDataVector: 
        """
        copy( (StdVec_JointDataVector)self) -> StdVec_JointDataVector :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_JointDataVector, arg2: object) -> None: 
        """
        extend( (StdVec_JointDataVector)arg1, (object)arg2) -> None
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_JointDataVector)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_JointModelVector(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_JointModelVector, arg2: object) -> bool: 
        """
        __contains__( (StdVec_JointModelVector)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_JointModelVector: 
        """
        __copy__( (StdVec_JointModelVector)self) -> StdVec_JointModelVector :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_JointModelVector: 
        """
        __deepcopy__( (StdVec_JointModelVector)self, (dict)memo) -> StdVec_JointModelVector :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_JointModelVector, arg2: object) -> None: 
        """
        __delitem__( (StdVec_JointModelVector)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_JointModelVector) -> tuple: 
        """
        __getinitargs__( (StdVec_JointModelVector)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_JointModelVector) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: JointModel) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_JointModelVector) -> int: 
        """
        __len__( (StdVec_JointModelVector)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_JointModelVector, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_JointModelVector)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_JointModelVector, arg2: object) -> None: 
        """
        append( (StdVec_JointModelVector)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_JointModelVector: 
        """
        copy( (StdVec_JointModelVector)self) -> StdVec_JointModelVector :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_JointModelVector, arg2: object) -> None: 
        """
        extend( (StdVec_JointModelVector)arg1, (object)arg2) -> None
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_JointModelVector)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_Matrix6(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Matrix6, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Matrix6)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_Matrix6: 
        """
        __copy__( (StdVec_Matrix6)self) -> StdVec_Matrix6 :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_Matrix6: 
        """
        __deepcopy__( (StdVec_Matrix6)self, (dict)memo) -> StdVec_Matrix6 :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Matrix6, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Matrix6)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_Matrix6) -> tuple: 
        """
        __getinitargs__( (StdVec_Matrix6)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_Matrix6) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: numpy.ndarray) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_Matrix6) -> int: 
        """
        __len__( (StdVec_Matrix6)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Matrix6, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Matrix6)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_Matrix6, arg2: object) -> None: 
        """
        append( (StdVec_Matrix6)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_Matrix6: 
        """
        copy( (StdVec_Matrix6)self) -> StdVec_Matrix6 :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_Matrix6, arg2: object) -> None: 
        """
        extend( (StdVec_Matrix6)arg1, (object)arg2) -> None
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_Matrix6)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_Matrix6)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_Matrix6x(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Matrix6x, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Matrix6x)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_Matrix6x: 
        """
        __copy__( (StdVec_Matrix6x)self) -> StdVec_Matrix6x :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_Matrix6x: 
        """
        __deepcopy__( (StdVec_Matrix6x)self, (dict)memo) -> StdVec_Matrix6x :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Matrix6x, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Matrix6x)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_Matrix6x) -> tuple: 
        """
        __getinitargs__( (StdVec_Matrix6x)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_Matrix6x) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: numpy.ndarray) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_Matrix6x) -> int: 
        """
        __len__( (StdVec_Matrix6x)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Matrix6x, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Matrix6x)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_Matrix6x, arg2: object) -> None: 
        """
        append( (StdVec_Matrix6x)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_Matrix6x: 
        """
        copy( (StdVec_Matrix6x)self) -> StdVec_Matrix6x :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_Matrix6x, arg2: object) -> None: 
        """
        extend( (StdVec_Matrix6x)arg1, (object)arg2) -> None
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_Matrix6x)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_MatrixXs(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_MatrixXs, arg2: object) -> bool: 
        """
        __contains__( (StdVec_MatrixXs)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_MatrixXs: 
        """
        __copy__( (StdVec_MatrixXs)self) -> StdVec_MatrixXs :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_MatrixXs: 
        """
        __deepcopy__( (StdVec_MatrixXs)self, (dict)memo) -> StdVec_MatrixXs :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_MatrixXs, arg2: object) -> None: 
        """
        __delitem__( (StdVec_MatrixXs)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_MatrixXs) -> tuple: 
        """
        __getinitargs__( (StdVec_MatrixXs)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_MatrixXs) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: numpy.ndarray) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_MatrixXs) -> int: 
        """
        __len__( (StdVec_MatrixXs)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_MatrixXs, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_MatrixXs)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_MatrixXs, arg2: object) -> None: 
        """
        append( (StdVec_MatrixXs)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_MatrixXs: 
        """
        copy( (StdVec_MatrixXs)self) -> StdVec_MatrixXs :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_MatrixXs, arg2: object) -> None: 
        """
        extend( (StdVec_MatrixXs)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_MatrixXs)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_MatrixXs)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_MatrixXs)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_Motion(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Motion, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Motion)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_Motion: 
        """
        __copy__( (StdVec_Motion)self) -> StdVec_Motion :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_Motion: 
        """
        __deepcopy__( (StdVec_Motion)self, (dict)memo) -> StdVec_Motion :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Motion, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Motion)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_Motion) -> tuple: 
        """
        __getinitargs__( (StdVec_Motion)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_Motion) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: Motion) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_Motion) -> int: 
        """
        __len__( (StdVec_Motion)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Motion, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Motion)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_Motion, arg2: object) -> None: 
        """
        append( (StdVec_Motion)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_Motion: 
        """
        copy( (StdVec_Motion)self) -> StdVec_Motion :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_Motion, arg2: object) -> None: 
        """
        extend( (StdVec_Motion)arg1, (object)arg2) -> None
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_Motion)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_RigidConstraintData(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_RigidConstraintData, arg2: object) -> bool: 
        """
        __contains__( (StdVec_RigidConstraintData)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_RigidConstraintData: 
        """
        __copy__( (StdVec_RigidConstraintData)self) -> StdVec_RigidConstraintData :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_RigidConstraintData: 
        """
        __deepcopy__( (StdVec_RigidConstraintData)self, (dict)memo) -> StdVec_RigidConstraintData :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_RigidConstraintData, arg2: object) -> None: 
        """
        __delitem__( (StdVec_RigidConstraintData)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_RigidConstraintData) -> tuple: 
        """
        __getinitargs__( (StdVec_RigidConstraintData)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_RigidConstraintData) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: RigidConstraintData) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_RigidConstraintData) -> int: 
        """
        __len__( (StdVec_RigidConstraintData)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_RigidConstraintData, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_RigidConstraintData)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_RigidConstraintData, arg2: object) -> None: 
        """
        append( (StdVec_RigidConstraintData)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_RigidConstraintData: 
        """
        copy( (StdVec_RigidConstraintData)self) -> StdVec_RigidConstraintData :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_RigidConstraintData, arg2: object) -> None: 
        """
        extend( (StdVec_RigidConstraintData)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_RigidConstraintData)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_RigidConstraintData)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_RigidConstraintData)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_RigidConstraintModel(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_RigidConstraintModel, arg2: object) -> bool: 
        """
        __contains__( (StdVec_RigidConstraintModel)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_RigidConstraintModel: 
        """
        __copy__( (StdVec_RigidConstraintModel)self) -> StdVec_RigidConstraintModel :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_RigidConstraintModel: 
        """
        __deepcopy__( (StdVec_RigidConstraintModel)self, (dict)memo) -> StdVec_RigidConstraintModel :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_RigidConstraintModel, arg2: object) -> None: 
        """
        __delitem__( (StdVec_RigidConstraintModel)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_RigidConstraintModel) -> tuple: 
        """
        __getinitargs__( (StdVec_RigidConstraintModel)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_RigidConstraintModel) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: RigidConstraintModel) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_RigidConstraintModel) -> int: 
        """
        __len__( (StdVec_RigidConstraintModel)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_RigidConstraintModel, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_RigidConstraintModel)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_RigidConstraintModel, arg2: object) -> None: 
        """
        append( (StdVec_RigidConstraintModel)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_RigidConstraintModel: 
        """
        copy( (StdVec_RigidConstraintModel)self) -> StdVec_RigidConstraintModel :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_RigidConstraintModel, arg2: object) -> None: 
        """
        extend( (StdVec_RigidConstraintModel)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_RigidConstraintModel)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_RigidConstraintModel)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_RigidConstraintModel)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_SE3(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_SE3, arg2: object) -> bool: 
        """
        __contains__( (StdVec_SE3)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_SE3: 
        """
        __copy__( (StdVec_SE3)self) -> StdVec_SE3 :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_SE3: 
        """
        __deepcopy__( (StdVec_SE3)self, (dict)memo) -> StdVec_SE3 :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_SE3, arg2: object) -> None: 
        """
        __delitem__( (StdVec_SE3)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_SE3) -> tuple: 
        """
        __getinitargs__( (StdVec_SE3)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_SE3) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: SE3) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_SE3) -> int: 
        """
        __len__( (StdVec_SE3)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_SE3, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_SE3)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_SE3, arg2: object) -> None: 
        """
        append( (StdVec_SE3)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_SE3: 
        """
        copy( (StdVec_SE3)self) -> StdVec_SE3 :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_SE3, arg2: object) -> None: 
        """
        extend( (StdVec_SE3)arg1, (object)arg2) -> None
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_SE3)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_Scalar():
    pass

class StdVec_StdString(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_StdString, arg2: object) -> bool: 
        """
        __contains__( (StdVec_StdString)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_StdString: 
        """
        __copy__( (StdVec_StdString)self) -> StdVec_StdString :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_StdString: 
        """
        __deepcopy__( (StdVec_StdString)self, (dict)memo) -> StdVec_StdString :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_StdString, arg2: object) -> None: 
        """
        __delitem__( (StdVec_StdString)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_StdString) -> tuple: 
        """
        __getinitargs__( (StdVec_StdString)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_StdString) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: str) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_StdString) -> int: 
        """
        __len__( (StdVec_StdString)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_StdString, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_StdString)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_StdString, arg2: object) -> None: 
        """
        append( (StdVec_StdString)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_StdString: 
        """
        copy( (StdVec_StdString)self) -> StdVec_StdString :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_StdString, arg2: object) -> None: 
        """
        extend( (StdVec_StdString)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_StdString)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_StdString)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_StdString)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_Symmetric3(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Symmetric3, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Symmetric3)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_Symmetric3: 
        """
        __copy__( (StdVec_Symmetric3)self) -> StdVec_Symmetric3 :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_Symmetric3: 
        """
        __deepcopy__( (StdVec_Symmetric3)self, (dict)memo) -> StdVec_Symmetric3 :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Symmetric3, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Symmetric3)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_Symmetric3) -> tuple: 
        """
        __getinitargs__( (StdVec_Symmetric3)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_Symmetric3) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: Symmetric3) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_Symmetric3) -> int: 
        """
        __len__( (StdVec_Symmetric3)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Symmetric3, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Symmetric3)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_Symmetric3, arg2: object) -> None: 
        """
        append( (StdVec_Symmetric3)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_Symmetric3: 
        """
        copy( (StdVec_Symmetric3)self) -> StdVec_Symmetric3 :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_Symmetric3, arg2: object) -> None: 
        """
        extend( (StdVec_Symmetric3)arg1, (object)arg2) -> None
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_Symmetric3)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_Vector3(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Vector3, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Vector3)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_Vector3: 
        """
        __copy__( (StdVec_Vector3)self) -> StdVec_Vector3 :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_Vector3: 
        """
        __deepcopy__( (StdVec_Vector3)self, (dict)memo) -> StdVec_Vector3 :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Vector3, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Vector3)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_Vector3) -> tuple: 
        """
        __getinitargs__( (StdVec_Vector3)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_Vector3) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: numpy.ndarray) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_Vector3) -> int: 
        """
        __len__( (StdVec_Vector3)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Vector3, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Vector3)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_Vector3, arg2: object) -> None: 
        """
        append( (StdVec_Vector3)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_Vector3: 
        """
        copy( (StdVec_Vector3)self) -> StdVec_Vector3 :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_Vector3, arg2: object) -> None: 
        """
        extend( (StdVec_Vector3)arg1, (object)arg2) -> None
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_Vector3)self [, (bool)deep_copy=False]) -> list :
            Returns the aligned_vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_VectorXb(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_VectorXb, arg2: object) -> bool: 
        """
        __contains__( (StdVec_VectorXb)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_VectorXb: 
        """
        __copy__( (StdVec_VectorXb)self) -> StdVec_VectorXb :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_VectorXb: 
        """
        __deepcopy__( (StdVec_VectorXb)self, (dict)memo) -> StdVec_VectorXb :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_VectorXb, arg2: object) -> None: 
        """
        __delitem__( (StdVec_VectorXb)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_VectorXb) -> tuple: 
        """
        __getinitargs__( (StdVec_VectorXb)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_VectorXb) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: numpy.ndarray) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_VectorXb) -> int: 
        """
        __len__( (StdVec_VectorXb)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_VectorXb, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_VectorXb)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_VectorXb, arg2: object) -> None: 
        """
        append( (StdVec_VectorXb)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_VectorXb: 
        """
        copy( (StdVec_VectorXb)self) -> StdVec_VectorXb :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_VectorXb, arg2: object) -> None: 
        """
        extend( (StdVec_VectorXb)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_VectorXb)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_VectorXb)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_VectorXb)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class StdVec_int(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_int, arg2: object) -> bool: 
        """
        __contains__( (StdVec_int)arg1, (object)arg2) -> bool
        """
    def __copy__(self) -> StdVec_int: 
        """
        __copy__( (StdVec_int)self) -> StdVec_int :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> StdVec_int: 
        """
        __deepcopy__( (StdVec_int)self, (dict)memo) -> StdVec_int :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __delitem__(arg1: StdVec_int, arg2: object) -> None: 
        """
        __delitem__( (StdVec_int)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(arg1: StdVec_int) -> tuple: 
        """
        __getinitargs__( (StdVec_int)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(arg1: object) -> tuple: 
        """
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @typing.overload
    def __init__(self, other: StdVec_int) -> None: ...
    @typing.overload
    def __init__(self, size: int, value: int) -> None: ...
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(arg1: StdVec_int) -> int: 
        """
        __len__( (StdVec_int)arg1) -> int
        """
    @staticmethod
    def __setitem__(arg1: StdVec_int, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_int)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(arg1: object, arg2: tuple) -> None: 
        """
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(arg1: StdVec_int, arg2: object) -> None: 
        """
        append( (StdVec_int)arg1, (object)arg2) -> None
        """
    def copy(self) -> StdVec_int: 
        """
        copy( (StdVec_int)self) -> StdVec_int :
            Returns a copy of *this.
        """
    @staticmethod
    def extend(arg1: StdVec_int, arg2: object) -> None: 
        """
        extend( (StdVec_int)arg1, (object)arg2) -> None
        """
    def id(self) -> int: 
        """
        id( (StdVec_int)self) -> int :
            Returns the unique identity of an object.
            For object held in C++, it corresponds to its memory address.
        """
    def reserve(self, new_cap: int) -> None: 
        """
        reserve( (StdVec_int)self, (int)new_cap) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_cap.
        """
    def tolist(self, deep_copy: bool = False) -> list: 
        """
        tolist( (StdVec_int)self [, (bool)deep_copy=False]) -> list :
            Returns the std::vector as a Python list.
        """
    __getstate_manages_dict__ = True
    __instance_size__ = 56
    __safe_for_unpickling__ = True
    pass

class Symmetric3(Boost.Python.instance):
    """
    This class represents symmetric 3x3 matrices.

    Supported operations ...
    """
    @staticmethod
    def Identity() -> Symmetric3: 
        """
        Identity() -> Symmetric3 :
            Returns identity matrix.
        """
    @staticmethod
    def Random() -> Symmetric3: 
        """
        Random() -> Symmetric3 :
            Returns a random symmetric 3x3 matrix.
        """
    @staticmethod
    def Zero() -> Symmetric3: 
        """
        Zero() -> Symmetric3 :
            Returns a zero 3x3 matrix.
        """
    @staticmethod
    @typing.overload
    def __add__(arg1: Symmetric3, arg2: Symmetric3) -> object: 
        """
        __add__( (Symmetric3)arg1, (Symmetric3)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __add__(arg1: Symmetric3, arg2: numpy.ndarray) -> object: ...
    def __copy__(self) -> Symmetric3: 
        """
        __copy__( (Symmetric3)self) -> Symmetric3 :
            Returns a copy of *this.
        """
    def __deepcopy__(self, memo: dict) -> Symmetric3: 
        """
        __deepcopy__( (Symmetric3)self, (dict)memo) -> Symmetric3 :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(arg1: Symmetric3, arg2: Symmetric3) -> object: 
        """
        __eq__( (Symmetric3)arg1, (Symmetric3)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(arg1: Symmetric3) -> tuple: 
        """
        __getinitargs__( (Symmetric3)arg1) -> tuple
        """
    @staticmethod
    def __iadd__(arg1: object, arg2: Symmetric3) -> object: 
        """
        __iadd__( (object)arg1, (Symmetric3)arg2) -> object
        """
    @staticmethod
    def __imul__(arg1: object, arg2: float) -> object: 
        """
        __imul__( (object)arg1, (float)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, clone: Symmetric3) -> object: 
        """
        __init__( (object)self) -> None :
            Default constructor.
        """
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, I: numpy.ndarray) -> None: ...
    @typing.overload
    def __init__(self, a0: float, a1: float, a2: float, a3: float, a4: float, a5: float) -> None: ...
    @typing.overload
    def __init__(self, other: Symmetric3) -> None: ...
    @staticmethod
    @typing.overload
    def __isub__(arg1: object, arg2: Symmetric3) -> object: 
        """
        __isub__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __isub__(arg1: object, arg2: object) -> object: ...
    @staticmethod
    def __mul__(arg1: Symmetric3, arg2: numpy.ndarray) -> object: 
        """
        __mul__( (Symmetric3)arg1, (numpy.ndarray)arg2) -> object
        """
    @staticmethod
    def __ne__(arg1: Symmetric3, arg2: Symmetric3) -> object: 
        """
        __ne__( (Symmetric3)arg1, (Symmetric3)arg2) -> object
        """
    @staticmethod
    def __repr__(arg1: Symmetric3) -> object: 
        """
        __repr__( (Symmetric3)arg1) -> object
        """
    @staticmethod
    def __str__(arg1: Symmetric3) -> object: 
        """
        __str__( (Symmetric3)arg1) -> object
        """
    @staticmethod
    @typing.overload
    def __sub__(arg1: Symmetric3, arg2: Symmetric3) -> object: 
        """
        __sub__( (Symmetric3)arg1, (object)arg2) -> object
        """
    @staticmethod
    @typing.overload
    def __sub__(arg1: Symmetric3, arg2: numpy.ndarray) -> object: ...
    @staticmethod
    @typing.overload
    def __sub__(arg1: Symmetric3, arg2: object) -> object: ...
    @staticmethod
    def cast(arg1: Symmetric3) -> Symmetric3: 
        """
        cast( (Symmetric3)arg1) -> Symmetric3 :
            Returns a cast of *this.
        """
    def copy(self) -> Symmetric3: 
        """
        copy( (Symmetric3)self) -> Symmetric3 :
            Returns a copy of *this.
        """
    def decomposeltI(self) -> object: 
        """
        decomposeltI( (Symmetric3)self) -> object :
            Computes L for a symmetric matrix S.
        """
    def fill(self, value: float) -> None: 
        """
        fill( (Symmetric3)self, (float)value) -> None
        """
    def inverse(self, res: numpy.ndarray) -> None: 
        """
        inverse( (Symmetric3)self, (numpy.ndarray)res) -> None :
            Invert the symmetrical 3x3 matrix.
        """
    def isApprox(self, other_: Symmetric3, prec: float = 1e-12) -> bool: 
        """
        isApprox( (Symmetric3)self, (Symmetric3)other [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to other, within the precision given by prec.
        """
    def isZero(self, prec: float = 1e-12) -> bool: 
        """
        isZero( (Symmetric3)self [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to the zero matrix, within the precision given by prec.
        """
    def matrix(self) -> numpy.ndarray: 
        """
        matrix( (Symmetric3)self) -> numpy.ndarray :
            Returns a matrix representation of the data.
        """
    @staticmethod
    def rhsMult(SE3: Symmetric3, vin: numpy.ndarray, vout: numpy.ndarray) -> None: 
        """
        rhsMult( (Symmetric3)SE3, (numpy.ndarray)vin, (numpy.ndarray)vout) -> None
        """
    def rotate(self, R: numpy.ndarray) -> Symmetric3: 
        """
        rotate( (Symmetric3)self, (numpy.ndarray)R) -> Symmetric3 :
            Computes R*S*R'
        """
    def setDiagonal(self, diag: numpy.ndarray) -> None: 
        """
        setDiagonal( (Symmetric3)self, (numpy.ndarray)diag) -> None :
            Set the diagonal elements of 3x3 matrix.
        """
    def setIdentity(self) -> None: 
        """
        setIdentity( (Symmetric3)self) -> None :
            Set the components of *this to identity.
        """
    def setRandom(self) -> None: 
        """
        setRandom( (Symmetric3)self) -> None :
            Set all the components of *this randomly.
        """
    def setZero(self) -> None: 
        """
        setZero( (Symmetric3)self) -> None :
            Set all the components of *this to zero.
        """
    @staticmethod
    def svx(v: Symmetric3, S3: numpy.ndarray) -> numpy.ndarray: 
        """
        svx( (Symmetric3)v, (numpy.ndarray)S3) -> numpy.ndarray :
            Performs the operation 
$ M = S_{3} [v]_{	imes} 
$.
        """
    def vtiv(self, v: numpy.ndarray) -> float: 
        """
        vtiv( (Symmetric3)self, (numpy.ndarray)v) -> float
        """
    @staticmethod
    def vxs(v: Symmetric3, S3: numpy.ndarray) -> numpy.ndarray: 
        """
        vxs( (Symmetric3)v, (numpy.ndarray)S3) -> numpy.ndarray :
            Performs the operation 
$ M = [v]_{	imes} S_{3} 
$., Apply the cross product of v on each column of S and return result matrix M.
        """
    @property
    def data(self) -> numpy.ndarray:
        """
        6D vector containing the data of the symmetric 3x3 matrix.

        :type: numpy.ndarray
        """
    __safe_for_unpickling__ = True
    pass

class TridiagonalSymmetricMatrix(Boost.Python.instance):
    """
    Tridiagonal symmetric matrix.
    """
    @staticmethod
    def __eq__(arg1: TridiagonalSymmetricMatrix, arg2: TridiagonalSymmetricMatrix) -> object: 
        """
        __eq__( (TridiagonalSymmetricMatrix)arg1, (TridiagonalSymmetricMatrix)arg2) -> object
        """
    def __init__(self, size: int) -> None: 
        """
        __init__( (object)self, (int)size) -> None :
            Default constructor from a given size.
        """
    @staticmethod
    def __mul__(arg1: TridiagonalSymmetricMatrix, arg2: numpy.ndarray) -> object: 
        """
        __mul__( (TridiagonalSymmetricMatrix)arg1, (numpy.ndarray)arg2) -> object
        """
    @staticmethod
    def __ne__(arg1: TridiagonalSymmetricMatrix, arg2: TridiagonalSymmetricMatrix) -> object: 
        """
        __ne__( (TridiagonalSymmetricMatrix)arg1, (TridiagonalSymmetricMatrix)arg2) -> object
        """
    @staticmethod
    def __rmul__(arg1: TridiagonalSymmetricMatrix, arg2: numpy.ndarray) -> object: 
        """
        __rmul__( (TridiagonalSymmetricMatrix)arg1, (numpy.ndarray)arg2) -> object
        """
    def cols(self) -> int: 
        """
        cols( (TridiagonalSymmetricMatrix)self) -> int
        """
    def diagonal(self) -> object: 
        """
        diagonal( (TridiagonalSymmetricMatrix)self) -> object :
            Reference of the diagonal elements of the symmetric tridiagonal matrix.
        """
    def isDiagonal(self, prec: float = 1e-12) -> bool: 
        """
        isDiagonal( (TridiagonalSymmetricMatrix)self [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to the a diagonal matrix, within the precision given by prec.
        """
    def isIdentity(self, prec: float = 1e-12) -> bool: 
        """
        isIdentity( (TridiagonalSymmetricMatrix)self [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to the identity matrix, within the precision given by prec.
        """
    def isZero(self, prec: float = 1e-12) -> bool: 
        """
        isZero( (TridiagonalSymmetricMatrix)self [, (float)prec=1e-12]) -> bool :
            Returns true if *this is approximately equal to the zero matrix, within the precision given by prec.
        """
    def matrix(self) -> numpy.ndarray: 
        """
        matrix( (TridiagonalSymmetricMatrix)self) -> numpy.ndarray
        """
    def rows(self) -> int: 
        """
        rows( (TridiagonalSymmetricMatrix)self) -> int
        """
    def setDiagonal(self, diagonal: numpy.ndarray) -> None: 
        """
        setDiagonal( (TridiagonalSymmetricMatrix)self, (numpy.ndarray)diagonal) -> None :
            Set the current tridiagonal matrix to a diagonal matrix given by the entry vector diagonal.
        """
    def setIdentity(self) -> None: 
        """
        setIdentity( (TridiagonalSymmetricMatrix)self) -> None :
            Set the current tridiagonal matrix to identity.
        """
    def setRandom(self) -> None: 
        """
        setRandom( (TridiagonalSymmetricMatrix)self) -> None :
            Set the current tridiagonal matrix to random.
        """
    def setZero(self) -> None: 
        """
        setZero( (TridiagonalSymmetricMatrix)self) -> None :
            Set the current tridiagonal matrix to zero.
        """
    def subDiagonal(self) -> object: 
        """
        subDiagonal( (TridiagonalSymmetricMatrix)self) -> object :
            Reference of the sub diagonal elements of the symmetric tridiagonal matrix.
        """
    pass

class boost_type_index(Boost.Python.instance):
    """
    The class type_index holds implementation-specific information about a type, including the name of the type and means to compare two types for equality or collating order.
    """
    @staticmethod
    def __eq__(arg1: boost_type_index, arg2: boost_type_index) -> object: 
        """
        __eq__( (boost_type_index)arg1, (boost_type_index)arg2) -> object
        """
    @staticmethod
    def __ge__(arg1: boost_type_index, arg2: boost_type_index) -> object: 
        """
        __ge__( (boost_type_index)arg1, (boost_type_index)arg2) -> object
        """
    @staticmethod
    def __gt__(arg1: boost_type_index, arg2: boost_type_index) -> object: 
        """
        __gt__( (boost_type_index)arg1, (boost_type_index)arg2) -> object
        """
    @staticmethod
    def __le__(arg1: boost_type_index, arg2: boost_type_index) -> object: 
        """
        __le__( (boost_type_index)arg1, (boost_type_index)arg2) -> object
        """
    @staticmethod
    def __lt__(arg1: boost_type_index, arg2: boost_type_index) -> object: 
        """
        __lt__( (boost_type_index)arg1, (boost_type_index)arg2) -> object
        """
    def hash_code(self) -> int: 
        """
        hash_code( (boost_type_index)self) -> int :
            Returns an unspecified value (here denoted by hash code) such that for all std::type_info objects referring to the same type, their hash code is the same.
        """
    def name(self) -> str: 
        """
        name( (boost_type_index)self) -> str :
            Returns an implementation defined null-terminated character string containing the name of the type. No guarantees are given; in particular, the returned string can be identical for several types and change between invocations of the same program.
        """
    def pretty_name(self) -> str: 
        """
        pretty_name( (boost_type_index)self) -> str :
            Human readible name.
        """
    pass

class map_indexing_suite_StdMap_String_VectorXd_entry(Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __repr__(arg1: map_indexing_suite_StdMap_String_VectorXd_entry) -> object: 
        """
        __repr__( (map_indexing_suite_StdMap_String_VectorXd_entry)arg1) -> object
        """
    @staticmethod
    def data(arg1: map_indexing_suite_StdMap_String_VectorXd_entry) -> object: 
        """
        data( (map_indexing_suite_StdMap_String_VectorXd_entry)arg1) -> object
        """
    @staticmethod
    def key(arg1: map_indexing_suite_StdMap_String_VectorXd_entry) -> str: 
        """
        key( (map_indexing_suite_StdMap_String_VectorXd_entry)arg1) -> str
        """
    __instance_size__ = 72
    pass

class std_type_index(Boost.Python.instance):
    """
    The class type_index holds implementation-specific information about a type, including the name of the type and means to compare two types for equality or collating order.
    """
    @staticmethod
    def __eq__(arg1: std_type_index, arg2: std_type_index) -> object: 
        """
        __eq__( (std_type_index)arg1, (std_type_index)arg2) -> object
        """
    @staticmethod
    def __ge__(arg1: std_type_index, arg2: std_type_index) -> object: 
        """
        __ge__( (std_type_index)arg1, (std_type_index)arg2) -> object
        """
    @staticmethod
    def __gt__(arg1: std_type_index, arg2: std_type_index) -> object: 
        """
        __gt__( (std_type_index)arg1, (std_type_index)arg2) -> object
        """
    @staticmethod
    def __le__(arg1: std_type_index, arg2: std_type_index) -> object: 
        """
        __le__( (std_type_index)arg1, (std_type_index)arg2) -> object
        """
    @staticmethod
    def __lt__(arg1: std_type_index, arg2: std_type_index) -> object: 
        """
        __lt__( (std_type_index)arg1, (std_type_index)arg2) -> object
        """
    def hash_code(self) -> int: 
        """
        hash_code( (std_type_index)self) -> int :
            Returns an unspecified value (here denoted by hash code) such that for all std::type_info objects referring to the same type, their hash code is the same.
        """
    def name(self) -> str: 
        """
        name( (std_type_index)self) -> str :
            Returns an implementation defined null-terminated character string containing the name of the type. No guarantees are given; in particular, the returned string can be identical for several types and change between invocations of the same program.
        """
    def pretty_name(self) -> str: 
        """
        pretty_name( (std_type_index)self) -> str :
            Human readible name.
        """
    pass

def Hlog3(R: numpy.ndarray, v: numpy.ndarray) -> numpy.ndarray:
    """
    Hlog3( (numpy.ndarray)R, (numpy.ndarray)v) -> numpy.ndarray :
        Vector v to be multiplied to the hessian
    """

def Jexp3(w: numpy.ndarray) -> numpy.ndarray:
    """
    Jexp3( (numpy.ndarray)w) -> numpy.ndarray :
        Jacobian of exp(v) which maps from the tangent of SO(3) at R = exp(v) to the tangent of SO(3) at Identity.
    """

@typing.overload
def Jexp6(motion: Motion) -> numpy.ndarray:
    """
    Jexp6( (Motion)motion) -> numpy.ndarray :
        Jacobian of exp(v) which maps from the tangent of SE(3) at exp(v) to the tangent of SE(3) at Identity.
    """
@typing.overload
def Jexp6(v: numpy.ndarray) -> numpy.ndarray:
    pass

def Jlog3(R: numpy.ndarray) -> numpy.ndarray:
    """
    Jlog3( (numpy.ndarray)R) -> numpy.ndarray :
        Jacobian of log(R) which maps from the tangent of SO(3) at R to the tangent of SO(3) at Identity.
    """

def Jlog6(M: SE3) -> numpy.ndarray:
    """
    Jlog6( (SE3)M) -> numpy.ndarray :
        Jacobian of log(M) which maps from the tangent of SE(3) at M to the tangent of SE(3) at Identity.
    """

def SE3ToXYZQUAT(arg1: SE3) -> numpy.ndarray:
    """
    SE3ToXYZQUAT( (SE3)arg1) -> numpy.ndarray :
        M
    """

def SE3ToXYZQUATtuple(arg1: SE3) -> tuple:
    """
    SE3ToXYZQUATtuple( (SE3)arg1) -> tuple :
        M
    """

@typing.overload
def XYZQUATToSE3(array: numpy.ndarray) -> SE3:
    """
    XYZQUATToSE3( (tuple)tuple) -> SE3 :
        Reverse function of SE3ToXYZQUAT: convert [X,Y,Z,x,y,z,w] to an SE3 element.
    """
@typing.overload
def XYZQUATToSE3(list: list) -> SE3:
    pass
@typing.overload
def XYZQUATToSE3(tuple: tuple) -> SE3:
    pass

@typing.overload
def aba(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, tau: numpy.ndarray, fext_: StdVec_Force, convention: Convention = pinocchio_pywrap_default.Convention.LOCAL) -> numpy.ndarray:
    """
    aba( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)tau [, (Convention)convention=pinocchio.pinocchio_pywrap_default.Convention.LOCAL]) -> numpy.ndarray :
        Compute ABA, store the result in data.ddq and return it.
        Parameters:
        	 model: Model of the kinematic tree
        	 data: Data related to the kinematic tree
        	 q: joint configuration (size model.nq)
        	 tau: joint velocity (size model.nv)
        	 v: joint torque (size model.nv)	 convention: Convention to use
    """
@typing.overload
def aba(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, tau_: numpy.ndarray, convention: Convention = pinocchio_pywrap_default.Convention.LOCAL) -> numpy.ndarray:
    pass

@typing.overload
def appendModel(modelA: Model, modelB: Model, frame_in_modelA: int, aMb: SE3) -> Model:
    """
    appendModel( (Model)modelA, (Model)modelB, (int)frame_in_modelA, (SE3)aMb) -> Model :
        Append a child model into a parent model, after a specific frame given by its index.
        
        Parameters:
        	modelA: the parent model
        	modelB: the child model
        	frameInModelA:  index of the frame of modelA where to append modelB
        	aMb: pose of modelB universe joint (index 0) in frameInModelA
        
    """
@typing.overload
def appendModel(modelA: Model, modelB: Model, geomModelA: GeometryModel, geomModelB: GeometryModel, frame_in_modelA: int, aMb: SE3) -> tuple:
    pass

def bodyRegressor(velocity: Motion, acceleration: Motion) -> numpy.ndarray:
    """
    bodyRegressor( (Motion)velocity, (Motion)acceleration) -> numpy.ndarray :
        Computes the regressor for the dynamic parameters of a single rigid body.
        The result is such that Ia + v x Iv = bodyRegressor(v,a) * I.toDynamicParameters()
        
        Parameters:
        	velocity: spatial velocity of the rigid body
        	acceleration: spatial acceleration of the rigid body
        
    """

@typing.overload
def buildGeomFromMJCF(model: Model, mjcf_filename: object, geom_type: GeometryType) -> GeometryModel:
    """
    buildGeomFromMJCF( (Model)model, (object)mjcf_filename, (GeometryType)geom_type) -> GeometryModel :
        Parse the Mjcf file given as input looking for the geometry of the given input model and
        return a GeometryModel containing either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).
        Parameters:
        	model: model of the robot
        	filename: path to the mjcf file containing the model of the robot
        	geom_type: type of geometry to extract from the mjcf file (either the VISUAL for display or the COLLISION for collision detection).
        
    """
@typing.overload
def buildGeomFromMJCF(model: Model, mjcf_filename: object, geom_type: GeometryType, mesh_loader: object) -> GeometryModel:
    pass

def buildGeomFromUrdf(model: Model, urdf_filename: object, geom_type_: GeometryType, geometry_model: object = None, package_dirs: object = None, mesh_loader: object = None) -> GeometryModel:
    """
    buildGeomFromUrdf( (Model)model, (object)urdf_filename, (GeometryType)geom_type [, (object)geometry_model=None [, (object)package_dirs=None [, (object)mesh_loader=None]]]) -> GeometryModel :
        Parse the URDF file given as input looking for the geometry of the given input model and
        and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in a GeometryModel object.
        Parameters:
        	model: model of the robot
        
        urdf_filename: path to the URDF file containing the model of the robot
        	geom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).
        	geometry_model: if provided, this geometry model will be used to store the parsed information instead of creating a new one
        	package_dirs: either a single path or a vector of paths pointing to folders containing the model of the robot
        	mesh_loader: unused because the Pinocchio is built without hpp-fcl
        
        Retuns:
        	a new GeometryModel if `geometry_model` is None else `geometry_model` (that has been updated).
        
    """

def buildGeomFromUrdfString(model: Model, urdf_string: str, geom_type_: GeometryType, geometry_model: object = None, package_dirs: object = None, mesh_loader: object = None) -> GeometryModel:
    """
    buildGeomFromUrdfString( (Model)model, (str)urdf_string, (GeometryType)geom_type [, (object)geometry_model=None [, (object)package_dirs=None [, (object)mesh_loader=None]]]) -> GeometryModel :
        Parse the URDF file given as input looking for the geometry of the given input model and
        and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in a GeometryModel object.
        Parameters:
        	model: model of the robot
        
        urdf_string: a string containing the URDF model of the robot
        	geom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).
        	geometry_model: if provided, this geometry model will be used to store the parsed information instead of creating a new one
        	package_dirs: either a single path or a vector of paths pointing to folders containing the model of the robot
        	mesh_loader: unused because the Pinocchio is built without hpp-fcl
        
        Retuns:
        	a new GeometryModel if `geometry_model` is None else `geometry_model` (that has been updated).
        
    """

@typing.overload
def buildModelFromMJCF(mjcf_filename: object) -> Model:
    """
    buildModelFromMJCF( (object)mjcf_filename) -> Model :
        Parse the MJCF file given in input and return a pinocchio Model.
    """
@typing.overload
def buildModelFromMJCF(mjcf_filename: object, root_joint: JointModel) -> Model:
    pass
@typing.overload
def buildModelFromMJCF(mjcf_filename: object, root_joint: JointModel, root_joint_name: str) -> tuple:
    pass

@typing.overload
def buildModelFromUrdf(urdf_filename: object) -> Model:
    """
    buildModelFromUrdf( (object)urdf_filename, (JointModel)root_joint) -> Model :
        Parse the URDF file given in input and return a pinocchio Model starting with the given root joint.
    """
@typing.overload
def buildModelFromUrdf(urdf_filename: object, model: Model) -> Model:
    pass
@typing.overload
def buildModelFromUrdf(urdf_filename: object, root_joint: JointModel) -> Model:
    pass
@typing.overload
def buildModelFromUrdf(urdf_filename: object, root_joint: JointModel, model: Model) -> Model:
    pass
@typing.overload
def buildModelFromUrdf(urdf_filename: object, root_joint: JointModel, root_joint_name: str) -> Model:
    pass
@typing.overload
def buildModelFromUrdf(urdf_filename: object, root_joint: JointModel, root_joint_name: str, model: Model) -> Model:
    pass

@typing.overload
def buildModelFromXML(urdf_xml_stream: str) -> Model:
    """
    buildModelFromXML( (str)urdf_xml_stream, (JointModel)root_joint) -> Model :
        Parse the URDF XML stream given in input and return a pinocchio Model starting with the given root joint.
    """
@typing.overload
def buildModelFromXML(urdf_xml_stream: str, model: Model) -> Model:
    pass
@typing.overload
def buildModelFromXML(urdf_xml_stream: str, root_joint: JointModel) -> Model:
    pass
@typing.overload
def buildModelFromXML(urdf_xml_stream: str, root_joint: JointModel, model: Model) -> Model:
    pass
@typing.overload
def buildModelFromXML(urdf_xml_stream: str, root_joint: JointModel, root_joint_name: str) -> Model:
    pass
@typing.overload
def buildModelFromXML(urdf_xml_stream: str, root_joint: JointModel, root_joint_name: str, model: Model) -> Model:
    pass




@typing.overload
def buildReducedModel(model: Model, geom_model: GeometryModel, list_of_joints_to_lock: StdVec_Index, reference_configuration: numpy.ndarray) -> tuple:
    """
    buildReducedModel( (Model)model, (StdVec_Index)list_of_joints_to_lock, (numpy.ndarray)reference_configuration) -> Model :
        Build a reduce model from a given input model and a list of joint to lock.
        
        Parameters:
        	model: input kinematic modell to reduce
        	list_of_joints_to_lock: list of joint indexes to lock
        	reference_configuration: reference configuration to compute the placement of the lock joints
        
    """
@typing.overload
def buildReducedModel(model: Model, list_of_geom_models: StdVec_GeometryModel, list_of_joints_to_lock: StdVec_Index, reference_configuration: numpy.ndarray) -> tuple:
    pass
@typing.overload
def buildReducedModel(model: Model, list_of_joints_to_lock: StdVec_Index, reference_configuration: numpy.ndarray) -> Model:
    pass

@typing.overload
def buildSampleModelHumanoid() -> Model:
    """
    buildSampleModelHumanoid() -> Model :
        Generate a (hard-coded) model of a simple humanoid.
    """
@typing.overload
def buildSampleModelHumanoid(using_free_flyer: bool) -> Model:
    pass

@typing.overload
def buildSampleModelHumanoidRandom() -> Model:
    """
    buildSampleModelHumanoidRandom() -> Model :
        Generate a (hard-coded) model of a humanoid robot with 6-DOF limbs and random joint placements.
        Only meant for unit tests.
    """
@typing.overload
def buildSampleModelHumanoidRandom(using_free_flyer: bool) -> Model:
    pass

def buildSampleModelManipulator() -> Model:
    """
    buildSampleModelManipulator() -> Model :
        Generate a (hard-coded) model of a simple manipulator.
    """

def ccrba(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> numpy.ndarray:
    """
    ccrba( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Computes the centroidal mapping, the centroidal momentum and the Centroidal Composite Rigid Body Inertia, puts the result in Data and returns the centroidal mapping.For the same price, it also computes the total joint jacobians (data.J).
    """

@typing.overload
def centerOfMass(model: Model, data: Data, kinematic_level_: KinematicLevel, compute_subtree_coms: bool = True) -> numpy.ndarray:
    """
    centerOfMass( (Model)model, (Data)data, (numpy.ndarray)q [, (bool)compute_subtree_coms=True]) -> numpy.ndarray :
        Compute the center of mass, putting the result in context::Data and return it.If compute_subtree_coms is True, the algorithm also computes the center of mass of the subtrees.
    """
@typing.overload
def centerOfMass(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, a_: numpy.ndarray, compute_subtree_coms: bool = True) -> numpy.ndarray:
    pass
@typing.overload
def centerOfMass(model: Model, data: Data, q: numpy.ndarray, v_: numpy.ndarray, compute_subtree_coms: bool = True) -> numpy.ndarray:
    pass
@typing.overload
def centerOfMass(model: Model, data: Data, q_: numpy.ndarray, compute_subtree_coms: bool = True) -> numpy.ndarray:
    pass
@typing.overload
def centerOfMass(model: Model, data_: Data, compute_subtree_coms: bool = True) -> numpy.ndarray:
    pass

def checkVersionAtLeast(major: int, minor: int, patch: int) -> bool:
    """
    checkVersionAtLeast( (int)major, (int)minor, (int)patch) -> bool :
        Checks if the current version of Pinocchio is at least the version provided by the input arguments.
    """

@typing.overload
def classicAcceleration(spatial_velocity: object, spatial_acceleration: object) -> numpy.ndarray:
    """
    classicAcceleration( (object)spatial_velocity, (object)spatial_acceleration) -> numpy.ndarray :
        Computes the classic acceleration from a given spatial velocity and spatial acceleration.
    """
@typing.overload
def classicAcceleration(spatial_velocity: object, spatial_acceleration: object, placement: SE3) -> numpy.ndarray:
    pass

@typing.overload
def computeABADerivatives(model: Model, data: Data) -> tuple:
    """
    computeABADerivatives( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)tau) -> tuple :
        Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv (aka ddq_dtau)
        which correspond to the partial derivatives of the joint acceleration vector output with respect to the joint configuration,
        velocity and torque vectors.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	tau: the joint torque vector (size model.nv)
        
        Returns: (ddq_dq, ddq_dv, ddq_da)
    """
@typing.overload
def computeABADerivatives(model: Model, data: Data, fext: StdVec_Force) -> tuple:
    pass
@typing.overload
def computeABADerivatives(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, tau: numpy.ndarray) -> tuple:
    pass
@typing.overload
def computeABADerivatives(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, tau: numpy.ndarray, fext: StdVec_Force) -> tuple:
    pass

def computeAllTerms(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> None:
    """
    computeAllTerms( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> None :
        Compute all the terms M, non linear effects, center of mass quantities, centroidal quantities and Jacobians inin the same loop and store the results in data.
        This algorithm is equivalent to calling:
        	- forwardKinematics
        	- crba
        	- nonLinearEffects
        	- computeJointJacobians
        	- centerOfMass
        	- jacobianCenterOfMass
        	- ccrba
        	- computeKineticEnergy
        	- computePotentialEnergy
        	- computeGeneralizedGravity
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """

def computeCentroidalDynamicsDerivatives(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, a: numpy.ndarray) -> tuple:
    """
    computeCentroidalDynamicsDerivatives( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> tuple :
        Computes the analytical derivatives of the centroidal dynamics
        with respect to the joint configuration vector, velocity and acceleration.
    """

def computeCentroidalMap(model: Model, data: Data, q: numpy.ndarray) -> numpy.ndarray:
    """
    computeCentroidalMap( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Computes the centroidal mapping, puts the result in Data.Ag and returns the centroidal mapping.
        For the same price, it also computes the total joint jacobians (data.J).
    """

def computeCentroidalMapTimeVariation(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> numpy.ndarray:
    """
    computeCentroidalMapTimeVariation( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Computes the time derivative of the centroidal momentum matrix Ag, puts the result in Data.Ag and returns the centroidal mapping.
        For the same price, it also computes the centroidal momentum matrix (data.Ag), the total joint jacobians (data.J) and the related joint jacobians time derivative (data.dJ)
    """

@typing.overload
def computeCentroidalMomentum(model: Model, data: Data) -> Force:
    """
    computeCentroidalMomentum( (Model)model, (Data)data) -> Force :
        Computes the Centroidal momentum, a.k.a. the total momentum of the system expressed around the center of mass.
    """
@typing.overload
def computeCentroidalMomentum(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> Force:
    pass

@typing.overload
def computeCentroidalMomentumTimeVariation(model: Model, data: Data) -> Force:
    """
    computeCentroidalMomentumTimeVariation( (Model)model, (Data)data) -> Force :
        Computes the Centroidal momentum and its time derivatives, a.k.a. the total momentum of the system and its time derivative expressed around the center of mass.
    """
@typing.overload
def computeCentroidalMomentumTimeVariation(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, a: numpy.ndarray) -> Force:
    pass

def computeComplementarityShift(cones: StdVec_CoulombFrictionCone, velocities: numpy.ndarray) -> numpy.ndarray:
    """
    computeComplementarityShift( (StdVec_CoulombFrictionCone)cones, (numpy.ndarray)velocities) -> numpy.ndarray :
        Compute the complementarity shift associated to the De Saxé function.
    """

def computeConeProjection(cones: StdVec_CoulombFrictionCone, forces: numpy.ndarray) -> numpy.ndarray:
    """
    computeConeProjection( (StdVec_CoulombFrictionCone)cones, (numpy.ndarray)forces) -> numpy.ndarray :
        Project a vector on the cartesian product of cones.
    """

def computeConstraintDynamicsDerivatives(model: Model, data: Data, contact_models: StdVec_RigidConstraintModel, contact_datas_: StdVec_RigidConstraintData, settings: ProximalSettings = ProximalSettings(1e-12, 1e-12, 0, 1)) -> tuple:
    """
    computeConstraintDynamicsDerivatives( (Model)model, (Data)data, (StdVec_RigidConstraintModel)contact_models, (StdVec_RigidConstraintData)contact_datas [, (ProximalSettings)settings=ProximalSettings(1e-12, 1e-12, 0, 1)]) -> tuple :
        Computes the derivatives of the forward dynamics with kinematic constraints (given in the list of constraint models).
        Assumes that constraintDynamics has been called first. See constraintDynamics for more details.
        This function returns the derivatives of joint acceleration (ddq) and contact forces (lambda_c) of the system with respect to q, v and tau.
        The output is a tuple with ddq_dq, ddq_dv, ddq_da, dlambda_dq, dlambda_dv, dlambda_da.
    """

def computeContactForces(model: Model, data: Data, c_ref: numpy.ndarray, contact_models: StdVec_RigidConstraintModel, contact_datas: StdVec_RigidConstraintData, cones: StdVec_CoulombFrictionCone, R: numpy.ndarray, constraint_correction: numpy.ndarray, settings_: ProximalSettings, lambda_guess: numpy.ndarray = None) -> numpy.ndarray:
    """
    computeContactForces( (Model)model, (Data)data, (numpy.ndarray)c_ref, (StdVec_RigidConstraintModel)contact_models, (StdVec_RigidConstraintData)contact_datas, (StdVec_CoulombFrictionCone)cones, (numpy.ndarray)R, (numpy.ndarray)constraint_correction, (ProximalSettings)settings [, (numpy.ndarray)lambda_guess=None]) -> numpy.ndarray :
        Compute the inverse dynamics with frictional contacts, store the result in Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	c_ref: the reference velocity of contact points
        	contact_models: list of contact models
        	contact_datas: list of contact datas
        	cones: list of friction cones
        	R: vector representing the diagonal of the compliance matrix
        	constraint_correction: vector representing the constraint correction
        	settings: the settings of the proximal algorithm
        	lambda_guess: initial guess for contact forces
        
    """

def computeCoriolisMatrix(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> numpy.ndarray:
    """
    computeCoriolisMatrix( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Compute the Coriolis Matrix C(q,v) of the Lagrangian dynamics, store the result in data.C and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """

def computeDampedDelassusMatrixInverse(arg1: Model, model: Data, data: numpy.ndarray, q: StdVec_RigidConstraintModel, contact_models: StdVec_RigidConstraintData, contact_datas_: float, mu: bool = 0) -> numpy.ndarray:
    """
    computeDampedDelassusMatrixInverse( (Model)arg1, (Data)model, (numpy.ndarray)data, (StdVec_RigidConstraintModel)q, (StdVec_RigidConstraintData)contact_models, (float)contact_datas [, (bool)mu=0]) -> numpy.ndarray :
        Computes the inverse of the Delassus matrix associated to a set of given constraints.
    """

def computeDelassusMatrix(model: Model, data: Data, q: numpy.ndarray, contact_models: StdVec_RigidConstraintModel, contact_datas_: StdVec_RigidConstraintData, mu: float = 0) -> numpy.ndarray:
    """
    computeDelassusMatrix( (Model)model, (Data)data, (numpy.ndarray)q, (StdVec_RigidConstraintModel)contact_models, (StdVec_RigidConstraintData)contact_datas [, (float)mu=0]) -> numpy.ndarray :
        Computes the Delassus matrix associated to a set of given constraints.
    """

def computeDualConeProjection(cones: StdVec_CoulombFrictionCone, velocities: numpy.ndarray) -> numpy.ndarray:
    """
    computeDualConeProjection( (StdVec_CoulombFrictionCone)cones, (numpy.ndarray)velocities) -> numpy.ndarray :
        Project a vector on the cartesian product of dual cones.
    """

def computeForwardKinematicsDerivatives(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, a: numpy.ndarray) -> None:
    """
    computeForwardKinematicsDerivatives( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> None :
        Computes all the terms required to compute the derivatives of the placement, spatial velocity and acceleration
        for any joint of the model.
        The results are stored in data.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        
    """

@typing.overload
def computeFrameJacobian(model: Model, data: Data, q: numpy.ndarray, frame_id: int) -> numpy.ndarray:
    """
    computeFrameJacobian( (Model)model, (Data)data, (numpy.ndarray)q, (int)frame_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the Jacobian of the frame given by its frame_id in the coordinate system given by reference_frame.
        
    """
@typing.overload
def computeFrameJacobian(model: Model, data: Data, q: numpy.ndarray, frame_id: int, reference_frame: ReferenceFrame) -> numpy.ndarray:
    pass

def computeFrameKinematicRegressor(model: Model, data: Data, frame_id: int, reference_frame: ReferenceFrame) -> numpy.ndarray:
    """
    computeFrameKinematicRegressor( (Model)model, (Data)data, (int)frame_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the kinematic regressor that links the joint placement variations of the whole kinematic tree to the placement variation of the frame given as input.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	frame_id: index of the frame
        	reference_frame: reference frame in which the result is expressed (LOCAL, LOCAL_WORLD_ALIGNED or WORLD)
        
    """

def computeGeneralizedGravity(model: Model, data: Data, q: numpy.ndarray) -> numpy.ndarray:
    """
    computeGeneralizedGravity( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Compute the generalized gravity contribution g(q) of the Lagrangian dynamics, store the result in data.g and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        
    """

def computeGeneralizedGravityDerivatives(model: Model, data: Data, q: numpy.ndarray) -> numpy.ndarray:
    """
    computeGeneralizedGravityDerivatives( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Computes the partial derivative of the generalized gravity contribution
        with respect to the joint configuration.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        Returns: dtau_statique_dq
        
    """

def computeImpulseDynamicsDerivatives(model: Model, data: Data, contact_models: StdVec_RigidConstraintModel, contact_datas_: StdVec_RigidConstraintData, r_coeff: float = 0, prox_settings: ProximalSettings = ProximalSettings(1e-12, 1e-12, 0, 1)) -> None:
    """
    computeImpulseDynamicsDerivatives( (Model)model, (Data)data, (StdVec_RigidConstraintModel)contact_models, (StdVec_RigidConstraintData)contact_datas [, (float)r_coeff=0 [, (ProximalSettings)prox_settings=ProximalSettings(1e-12, 1e-12, 0, 1)]]) -> None :
        Computes the impulse dynamics derivatives with contact constraints according to a given list of Contact information.
        impulseDynamics should have been called before.
    """

def computeJointJacobian(model: Model, data: Data, q: numpy.ndarray, joint_id: int) -> numpy.ndarray:
    """
    computeJointJacobian( (Model)model, (Data)data, (numpy.ndarray)q, (int)joint_id) -> numpy.ndarray :
        Computes the Jacobian of a specific joint frame expressed in the local frame of the joint according to the given input configuration.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	joint_id: index of the joint
        
    """

@typing.overload
def computeJointJacobians(model: Model, data: Data) -> numpy.ndarray:
    """
    computeJointJacobians( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Computes the full model Jacobian, i.e. the stack of all the motion subspaces expressed in the coordinate world frame.
        The result is accessible through data.J. This function computes also the forward kinematics of the model.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        
    """
@typing.overload
def computeJointJacobians(model: Model, data: Data, q: numpy.ndarray) -> numpy.ndarray:
    pass

def computeJointJacobiansTimeVariation(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> numpy.ndarray:
    """
    computeJointJacobiansTimeVariation( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Computes the full model Jacobian variations with respect to time. It corresponds to dJ/dt which depends both on q and v. It also computes the joint Jacobian of the model (similar to computeJointJacobians).The result is accessible through data.dJ and data.J.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """

@typing.overload
def computeJointKinematicRegressor(model: Model, data: Data, joint_id: int, reference_frame: ReferenceFrame) -> numpy.ndarray:
    """
    computeJointKinematicRegressor( (Model)model, (Data)data, (int)joint_id, (ReferenceFrame)reference_frame, (SE3)placement) -> numpy.ndarray :
        Computes the kinematic regressor that links the joint placements variations of the whole kinematic tree to the placement variation of the frame rigidly attached to the joint and given by its placement w.r.t. to the joint frame.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	reference_frame: reference frame in which the result is expressed (LOCAL, LOCAL_WORLD_ALIGNED or WORLD)
        	placement: relative placement to the joint frame
        
    """
@typing.overload
def computeJointKinematicRegressor(model: Model, data: Data, joint_id: int, reference_frame: ReferenceFrame, placement: SE3) -> numpy.ndarray:
    pass

def computeJointTorqueRegressor(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, a: numpy.ndarray) -> numpy.ndarray:
    """
    computeJointTorqueRegressor( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> numpy.ndarray :
        Compute the joint torque regressor that links the joint torque to the dynamic parameters of each link according to the current the robot motion,
        store the result in context::Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        
    """

def computeKKTContactDynamicMatrixInverse(model: Model, data: Data, q: numpy.ndarray, constraint_jacobian_: numpy.ndarray, damping: float = 0) -> numpy.ndarray:
    """
    computeKKTContactDynamicMatrixInverse( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)constraint_jacobian [, (float)damping=0]) -> numpy.ndarray :
        Computes the inverse of the constraint matrix [[M J^T], [J 0]].
    """

@typing.overload
def computeKineticEnergy(model: Model, data: Data) -> float:
    """
    computeKineticEnergy( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> float :
        Computes the forward kinematics and the kinematic energy of the system for the given joint configuration and velocity given as input. The result is accessible through data.kinetic_energy.
    """
@typing.overload
def computeKineticEnergy(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> float:
    pass

def computeKineticEnergyRegressor(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> numpy.ndarray:
    """
    computeKineticEnergyRegressor( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Compute the kinetic energy regressor that links the kinetic energyto the dynamic parameters of each link according to the current the robot motion,
        store the result in context::Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """

@typing.overload
def computeMechanicalEnergy(model: Model, data: Data) -> float:
    """
    computeMechanicalEnergy( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> float :
        Computes the forward kinematics and the kinematic energy of the system for the given joint configuration and velocity given as input. The result is accessible through data.mechanical_energy.
        A byproduct of this function is the computation of both data.kinetic_energy and data.potential_energy too.
    """
@typing.overload
def computeMechanicalEnergy(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> float:
    pass

@typing.overload
def computeMinverse(model: Model, data: Data) -> numpy.ndarray:
    """
    computeMinverse( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Computes the inverse of the joint space inertia matrix using an extension of the Articulated Body algorithm.
        The result is stored in data.Minv.
        Parameters:
        	 model: Model of the kinematic tree
        	 data: Data related to the kinematic tree
        	 q: joint configuration (size model.nq)
    """
@typing.overload
def computeMinverse(model: Model, data: Data, q: numpy.ndarray) -> numpy.ndarray:
    pass

@typing.overload
def computePotentialEnergy(model: Model, data: Data) -> float:
    """
    computePotentialEnergy( (Model)model, (Data)data, (numpy.ndarray)q) -> float :
        Computes the potential energy of the system for the given the joint configuration given as input. The result is accessible through data.potential_energy.
    """
@typing.overload
def computePotentialEnergy(model: Model, data: Data, q: numpy.ndarray) -> float:
    pass

def computePotentialEnergyRegressor(model: Model, data: Data, q: numpy.ndarray) -> numpy.ndarray:
    """
    computePotentialEnergyRegressor( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Compute the potential energy regressor that links the potential energyto the dynamic parameters of each link according to the current the robot motion,
        store the result in context::Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        
    """

def computePrimalFeasibility(cones: StdVec_CoulombFrictionCone, forces: numpy.ndarray) -> float:
    """
    computePrimalFeasibility( (StdVec_CoulombFrictionCone)cones, (numpy.ndarray)forces) -> float :
        Compute the primal feasibility.
    """

@typing.overload
def computeRNEADerivatives(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, a: numpy.ndarray) -> tuple:
    """
    computeRNEADerivatives( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> tuple :
        Computes the RNEA partial derivatives, store the result in data.dtau_dq, data.dtau_dv and data.M (aka dtau_da)
        which correspond to the partial derivatives of the torque output with respect to the joint configuration,
        velocity and acceleration vectors.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        
        Returns: (dtau_dq, dtau_dv, dtau_da)
        
    """
@typing.overload
def computeRNEADerivatives(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, a: numpy.ndarray, fext: StdVec_Force) -> tuple:
    pass

def computeReprojectionError(cones: StdVec_CoulombFrictionCone, forces: numpy.ndarray, velocities: numpy.ndarray) -> float:
    """
    computeReprojectionError( (StdVec_CoulombFrictionCone)cones, (numpy.ndarray)forces, (numpy.ndarray)velocities) -> float :
        Compute the reprojection error.
    """

def computeStaticRegressor(model: Model, data: Data, q: numpy.ndarray) -> numpy.ndarray:
    """
    computeStaticRegressor( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Compute the static regressor that links the inertia parameters of the system to its center of mass position,
        store the result in context::Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        
    """

def computeStaticTorque(model: Model, data: Data, q: numpy.ndarray, fext: StdVec_Force) -> numpy.ndarray:
    """
    computeStaticTorque( (Model)model, (Data)data, (numpy.ndarray)q, (StdVec_Force)fext) -> numpy.ndarray :
        Computes the generalized static torque contribution g(q) - J.T f_ext of the Lagrangian dynamics, store the result in data.tau and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	fext: list of external forces expressed in the local frame of the joints (size model.njoints)
        
    """

def computeStaticTorqueDerivatives(model: Model, data: Data, q: numpy.ndarray, fext: StdVec_Force) -> numpy.ndarray:
    """
    computeStaticTorqueDerivatives( (Model)model, (Data)data, (numpy.ndarray)q, (StdVec_Force)fext) -> numpy.ndarray :
        Computes the partial derivative of the generalized gravity and external forces contributions (a.k.a static torque vector)
        with respect to the joint configuration.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	fext: list of external forces expressed in the local frame of the joints (size model.njoints)
        Returns: dtau_statique_dq
        
    """

def computeSubtreeMasses(model: Model, data: Data) -> None:
    """
    computeSubtreeMasses( (Model)model, (Data)data) -> None :
        Compute the mass of each kinematic subtree and store it in the vector data.mass.
    """

def computeSupportedForceByFrame(model: Model, data: Data, frame_id: int) -> Force:
    """
    computeSupportedForceByFrame( (Model)model, (Data)data, (int)frame_id) -> Force :
        Computes the supported force of the frame (given by frame_id) and returns it.
        The supported force corresponds to the sum of all the forces experienced after the given frame.
        You must first call pinocchio::rnea to update placement values in data structure.
    """

def computeSupportedInertiaByFrame(model: Model, data: Data, frame_id: int, with_subtree: bool) -> Inertia:
    """
    computeSupportedInertiaByFrame( (Model)model, (Data)data, (int)frame_id, (bool)with_subtree) -> Inertia :
        Computes the supported inertia by the frame (given by frame_id) and returns it.
        The supported inertia corresponds to the sum of the inertias of all the child frames (that belongs to the same joint body) and the child joints, if with_subtree=True.
        You must first call pinocchio::forwardKinematics to update placement values in data structure.
    """

@typing.overload
def computeTotalMass(model: Model) -> float:
    """
    computeTotalMass( (Model)model) -> float :
        Compute the total mass of the model and return it.
    """
@typing.overload
def computeTotalMass(model: Model, data: Data) -> float:
    pass

def constraintDynamics(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, tau: numpy.ndarray, contact_models: StdVec_RigidConstraintModel, contact_datas_: StdVec_RigidConstraintData, prox_settings: ProximalSettings) -> numpy.ndarray:
    """
    constraintDynamics( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)tau, (StdVec_RigidConstraintModel)contact_models, (StdVec_RigidConstraintData)contact_datas [, (ProximalSettings)prox_settings]) -> numpy.ndarray :
        Computes the forward dynamics with contact constraints according to a given list of Contact information.
        When using constraintDynamics for the first time, you should call first initConstraintDynamics to initialize the internal memory used in the algorithm.
        This function returns joint acceleration of the system. The contact forces are stored in the list data.contact_forces.
    """

def contactInverseDynamics(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, a: numpy.ndarray, dt: float, contact_models: StdVec_RigidConstraintModel, contact_datas: StdVec_RigidConstraintData, cones: StdVec_CoulombFrictionCone, R: numpy.ndarray, constraint_correction: numpy.ndarray, settings_: ProximalSettings, lambda_guess: numpy.ndarray = None) -> numpy.ndarray:
    """
    contactInverseDynamics( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a, (float)dt, (StdVec_RigidConstraintModel)contact_models, (StdVec_RigidConstraintData)contact_datas, (StdVec_CoulombFrictionCone)cones, (numpy.ndarray)R, (numpy.ndarray)constraint_correction, (ProximalSettings)settings [, (numpy.ndarray)lambda_guess=None]) -> numpy.ndarray :
        Compute the inverse dynamics with frictional contacts, store the result in Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        	dt: the time step
        	contact_models: list of contact models
        	contact_datas: list of contact datas
        	cones: list of friction cones
        	R: vector representing the diagonal of the compliance matrix
        	constraint_correction: vector representing the constraint correction
        	settings: the settings of the proximal algorithm
        	lambda_guess: initial guess for contact forces
        
    """

def crba(model: Model, data: Data, q_: numpy.ndarray, convention: Convention = pinocchio_pywrap_default.Convention.LOCAL) -> numpy.ndarray:
    """
    crba( (Model)model, (Data)data, (numpy.ndarray)q [, (Convention)convention=pinocchio.pinocchio_pywrap_default.Convention.LOCAL]) -> numpy.ndarray :
        Computes CRBA, store the result in Data and return it.
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	 convention: Convention to use
    """


@typing.overload
def dDifference(model: Model, q1: numpy.ndarray, q2: numpy.ndarray) -> tuple:
    """
    dDifference( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2) -> tuple :
        Computes the partial derivatives of the difference function with respect to the first and the second argument, and returns the two Jacobians as a tuple.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: the initial joint configuration vector (size model.nq)
        	q2: the terminal joint configuration vector (size model.nq)
        
    """
@typing.overload
def dDifference(model: Model, q1: numpy.ndarray, q2: numpy.ndarray, argument_position: ArgumentPosition) -> numpy.ndarray:
    pass

@typing.overload
def dIntegrate(model: Model, q: numpy.ndarray, v: numpy.ndarray) -> tuple:
    """
    dIntegrate( (Model)model, (numpy.ndarray)q, (numpy.ndarray)v) -> tuple :
        Computes the partial derivatives of the integrate function with respect to the first and the second argument, and returns the two Jacobians as a tuple.
        
        Parameters:
        	model: model of the kinematic tree
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """
@typing.overload
def dIntegrate(model: Model, q: numpy.ndarray, v: numpy.ndarray, argument_position: ArgumentPosition) -> numpy.ndarray:
    pass

def dIntegrateTransport(model: Model, q: numpy.ndarray, v: numpy.ndarray, Jin: numpy.ndarray, argument_position: ArgumentPosition) -> numpy.ndarray:
    """
    dIntegrateTransport( (Model)model, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)Jin, (ArgumentPosition)argument_position) -> numpy.ndarray :
        Takes a matrix expressed at q (+) v and uses parallel transport to express it in the tangent space at q.	This operation does the product of the matrix by the Jacobian of the integration operation, but more efficiently.Parameters:
        	model: model of the kinematic tree
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	Jin: the input matrix (row size model.nv)	argument_position: either pinocchio.ArgumentPosition.ARG0 (q) or pinocchio.ArgumentPosition.ARG1 (v), depending on the desired Jacobian value.
        
    """

def dccrba(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> numpy.ndarray:
    """
    dccrba( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Computes the time derivative of the centroidal momentum matrix Ag in terms of q and v.
        For the same price, it also computes the centroidal momentum matrix (data.Ag), the total joint jacobians (data.J) and the related joint jacobians time derivative (data.dJ)
    """

def difference(model: Model, q1: numpy.ndarray, q2: numpy.ndarray) -> numpy.ndarray:
    """
    difference( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2) -> numpy.ndarray :
        Difference between two joint configuration vectors, i.e. the tangent vector that must be integrated during one unit timeto go from q1 to q2.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: the initial joint configuration vector (size model.nq)
        	q2: the terminal joint configuration vector (size model.nq)
        
    """

def distance(model: Model, q1: numpy.ndarray, q2: numpy.ndarray) -> float:
    """
    distance( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2) -> float :
        Distance between two joint configuration vectors.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: the initial joint configuration vector (size model.nq)
        	q2: the terminal joint configuration vector (size model.nq)
        
    """


def exp3(w: numpy.ndarray) -> numpy.ndarray:
    """
    exp3( (numpy.ndarray)w) -> numpy.ndarray :
        Exp: so3 -> SO3. Return the integral of the input vector w during time 1. This is also known as the Rodrigues formula.
    """

def exp3_quat(w: numpy.ndarray) -> numpy.ndarray:
    """
    exp3_quat( (numpy.ndarray)w) -> numpy.ndarray :
        Exp: so3 -> S3. Returns the integral of the input vector w during time 1, represented as a unit Quaternion.
    """

@typing.overload
def exp6(motion: Motion) -> SE3:
    """
    exp6( (Motion)motion) -> SE3 :
        Exp: se3 -> SE3. Return the integral of the input spatial velocity during time 1.
    """
@typing.overload
def exp6(v: numpy.ndarray) -> SE3:
    pass

def exp6_quat(v: numpy.ndarray) -> numpy.ndarray:
    """
    exp6_quat( (numpy.ndarray)v) -> numpy.ndarray :
        Exp: se3 -> R3 * S3. Return the integral of the input 6D spatial velocity over unit time, using quaternion to represent rotation as in the standard configuration layout for the Lie group SE3.
    """

def findCommonAncestor(model: Model, joint1_id: int, joint2_id: int) -> tuple:
    """
    findCommonAncestor( (Model)model, (int)joint1_id, (int)joint2_id) -> tuple :
        Computes the common ancestor between two joints belonging to the same kinematic tree.
        
        Parameters:
        	model: input model
        	joint1_id: index of the first joint
        	joint2_id: index of the second joint
        Returns a tuple containing the index of the common joint ancestor, the position of this ancestor in model.supports[joint1_id] and model.supports[joint2_id].
        
    """

@typing.overload
def forwardDynamics(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, tau: numpy.ndarray, constraint_jacobian: numpy.ndarray, constraint_drift_: numpy.ndarray, damping: float = 0) -> numpy.ndarray:
    """
    forwardDynamics( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)tau, (numpy.ndarray)constraint_jacobian, (numpy.ndarray)constraint_drift [, (float)damping=0]) -> numpy.ndarray :
        Solves the constrained dynamics problem with contacts, puts the result in context::Data::ddq and return it. The contact forces are stored in data.lambda_c.
        Note: internally, pinocchio.computeAllTerms is called.
    """
@typing.overload
def forwardDynamics(model: Model, data: Data, tau: numpy.ndarray, constraint_jacobian: numpy.ndarray, constraint_drift_: numpy.ndarray, damping: float = 0) -> numpy.ndarray:
    pass

@typing.overload
def forwardKinematics(model: Model, data: Data, q: numpy.ndarray) -> None:
    """
    forwardKinematics( (Model)model, (Data)data, (numpy.ndarray)q) -> None :
        Compute the global placements of all the joints of the kinematic tree and store the results in data.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        
    """
@typing.overload
def forwardKinematics(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> None:
    pass
@typing.overload
def forwardKinematics(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, a: numpy.ndarray) -> None:
    pass

def frameBodyRegressor(model: Model, data: Data, frame_id: int) -> numpy.ndarray:
    """
    frameBodyRegressor( (Model)model, (Data)data, (int)frame_id) -> numpy.ndarray :
        Computes the regressor for the dynamic parameters of a rigid body attached to a given frame.
        This algorithm assumes RNEA has been run to compute the acceleration and gravitational effects.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	frame_id: index of the frame
        
    """

def frameJacobianTimeVariation(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, frame_id: int, reference_frame: ReferenceFrame) -> numpy.ndarray:
    """
    frameJacobianTimeVariation( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (int)frame_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the Jacobian Time Variation of the frame given by its frame_id either in the reference frame provided by reference_frame.
        
    """

def framesForwardKinematics(model: Model, data: Data, q: numpy.ndarray) -> None:
    """
    framesForwardKinematics( (Model)model, (Data)data, (numpy.ndarray)q) -> None :
        Calls first the forwardKinematics(model,data,q) and then update the Frame placement quantities (data.oMf).
    """

def getAcceleration(model: Model, data: Data, joint_id_: int, reference_frame: ReferenceFrame = pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> Motion:
    """
    getAcceleration( (Model)model, (Data)data, (int)joint_id [, (ReferenceFrame)reference_frame=pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL]) -> Motion :
        Returns the spatial acceleration of the joint expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial acceleration stored in data.a .
    """

def getCenterOfMassVelocityDerivatives(model: Model, data: Data) -> numpy.ndarray:
    """
    getCenterOfMassVelocityDerivatives( (Model)model, (Data)data) -> numpy.ndarray :
        Computes the partial derivaties of the center of mass velocity with respect to
        the joint configuration.
        You must first call computeAllTerms(model,data,q,v) or centerOfMass(model,data,q,v) before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        
    """

def getCentroidalDynamicsDerivatives(model: Model, data: Data) -> tuple:
    """
    getCentroidalDynamicsDerivatives( (Model)model, (Data)data) -> tuple :
        Retrive the analytical derivatives of the centroidal dynamics
        from the RNEA derivatives.
        pinocchio.computeRNEADerivatives should have been called first.
    """

def getClassicalAcceleration(model: Model, data: Data, joint_id_: int, reference_frame: ReferenceFrame = pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> Motion:
    """
    getClassicalAcceleration( (Model)model, (Data)data, (int)joint_id [, (ReferenceFrame)reference_frame=pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL]) -> Motion :
        Returns the "classical" acceleration of the joint expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial acceleration stored in data.a .
    """

def getConstraintJacobian(model: Model, data: Data, contact_model: RigidConstraintModel, contact_data: RigidConstraintData) -> numpy.ndarray:
    """
    getConstraintJacobian( (Model)model, (Data)data, (RigidConstraintModel)contact_model, (RigidConstraintData)contact_data) -> numpy.ndarray :
        Computes the kinematic Jacobian associatied to a given constraint model.
    """

def getConstraintsJacobian(model: Model, data: Data, contact_models: StdVec_RigidConstraintModel, contact_datas: StdVec_RigidConstraintData) -> numpy.ndarray:
    """
    getConstraintsJacobian( (Model)model, (Data)data, (StdVec_RigidConstraintModel)contact_models, (StdVec_RigidConstraintData)contact_datas) -> numpy.ndarray :
        Computes the kinematic Jacobian associatied to a given set of constraint models.
    """

def getCoriolisMatrix(model: Model, data: Data) -> numpy.ndarray:
    """
    getCoriolisMatrix( (Model)model, (Data)data) -> numpy.ndarray :
        Retrives the Coriolis Matrix C(q,v) of the Lagrangian dynamics after calling one of the derivative algorithms, store the result in data.C and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        
    """

@typing.overload
def getFrameAcceleration(model: Model, data: Data, frame_id_: int, reference_frame: ReferenceFrame = pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> Motion:
    """
    getFrameAcceleration( (Model)model, (Data)data, (int)frame_id [, (ReferenceFrame)reference_frame=pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL]) -> Motion :
        Returns the spatial acceleration of the frame expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial acceleration stored in data.a .
    """
@typing.overload
def getFrameAcceleration(model: Model, data: Data, joint_id: int, placement_: SE3, reference_frame: ReferenceFrame = pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> Motion:
    pass

@typing.overload
def getFrameAccelerationDerivatives(model: Model, data: Data, frame_id: int, reference_frame: ReferenceFrame) -> tuple:
    """
    getFrameAccelerationDerivatives( (Model)model, (Data)data, (int)frame_id, (ReferenceFrame)reference_frame) -> tuple :
        Computes the partial derivatives of the spatial acceleration of a given frame with respect to
        the joint configuration, velocity and acceleration and returns them as a tuple.
        The partial derivatives can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.
        You must first call computeForwardKinematicsDerivatives before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	frame_id: index of the frame
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """
@typing.overload
def getFrameAccelerationDerivatives(model: Model, data: Data, joint_id: int, placement: SE3, reference_frame: ReferenceFrame) -> tuple:
    pass

@typing.overload
def getFrameClassicalAcceleration(model: Model, data: Data, frame_id_: int, reference_frame: ReferenceFrame = pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> Motion:
    """
    getFrameClassicalAcceleration( (Model)model, (Data)data, (int)frame_id [, (ReferenceFrame)reference_frame=pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL]) -> Motion :
        Returns the "classical" acceleration of the frame expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial acceleration stored in data.a .
    """
@typing.overload
def getFrameClassicalAcceleration(model: Model, data: Data, joint_id: int, placement_: SE3, reference_frame: ReferenceFrame = pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> Motion:
    pass

@typing.overload
def getFrameJacobian(model: Model, data: Data, frame_id: int, reference_frame: ReferenceFrame) -> numpy.ndarray:
    """
    getFrameJacobian( (Model)model, (Data)data, (int)frame_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the Jacobian of the frame given by its ID either in the LOCAL, LOCAL_WORLD_ALIGNED or the WORLD coordinates systems.
        In other words, the velocity of the frame vF expressed in the reference frame is given by J*v,where v is the joint velocity vector.
        remarks: computeJointJacobians(model,data,q) must have been called first.
    """
@typing.overload
def getFrameJacobian(model: Model, data: Data, joint_id: int, placement: SE3, reference_frame: ReferenceFrame) -> numpy.ndarray:
    pass

def getFrameJacobianTimeVariation(model: Model, data: Data, frame_id: int, reference_frame: ReferenceFrame) -> numpy.ndarray:
    """
    getFrameJacobianTimeVariation( (Model)model, (Data)data, (int)frame_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Returns the Jacobian time variation of the frame given by its frame_id either in the reference frame provided by reference_frame.
        You have to call computeJointJacobiansTimeVariation(model,data,q,v) and updateFramePlacements(model,data) first.
    """

@typing.overload
def getFrameVelocity(model: Model, data: Data, frame_id_: int, reference_frame: ReferenceFrame = pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> Motion:
    """
    getFrameVelocity( (Model)model, (Data)data, (int)frame_id [, (ReferenceFrame)reference_frame=pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL]) -> Motion :
        Returns the spatial velocity of the frame expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v[,a]) should be called first to compute the joint spatial velocity stored in data.v
    """
@typing.overload
def getFrameVelocity(model: Model, data: Data, joint_id: int, placement_: SE3, reference_frame: ReferenceFrame = pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> Motion:
    pass

@typing.overload
def getFrameVelocityDerivatives(model: Model, data: Data, frame_id: int, reference_frame: ReferenceFrame) -> tuple:
    """
    getFrameVelocityDerivatives( (Model)model, (Data)data, (int)frame_id, (ReferenceFrame)reference_frame) -> tuple :
        Computes the partial derivatives of the spatial velocity of a given frame with respect to
        the joint configuration and velocity and returns them as a tuple.
        The partial derivatives can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.
        You must first call computeForwardKinematicsDerivatives before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	frame_id: index of the frame
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """
@typing.overload
def getFrameVelocityDerivatives(model: Model, data: Data, joint_id: int, placement: SE3, reference_frame: ReferenceFrame) -> tuple:
    pass

def getJacobianSubtreeCenterOfMass(model: Model, data: Data, subtree_root_joint_id: int) -> numpy.ndarray:
    """
    getJacobianSubtreeCenterOfMass( (Model)model, (Data)data, (int)subtree_root_joint_id) -> numpy.ndarray :
        Get the Jacobian of the CoM of the given subtree expressed in the world frame, according to the given entries in data. It assumes that jacobianCenterOfMass has been called first.
    """

def getJointAccelerationDerivatives(model: Model, data: Data, joint_id: int, reference_frame: ReferenceFrame) -> tuple:
    """
    getJointAccelerationDerivatives( (Model)model, (Data)data, (int)joint_id, (ReferenceFrame)reference_frame) -> tuple :
        Computes the partial derivatives of the spatial acceleration of a given joint with respect to
        the joint configuration, velocity and acceleration and returns them as a tuple.
        The partial derivatives can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.
        You must first call computeForwardKinematicsDerivatives before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """

def getJointJacobian(model: Model, data: Data, joint_id: int, reference_frame: ReferenceFrame) -> numpy.ndarray:
    """
    getJointJacobian( (Model)model, (Data)data, (int)joint_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the jacobian of a given given joint according to the given entries in data.
        If reference_frame is set to LOCAL, it returns the Jacobian expressed in the local coordinate system of the joint.
        If reference_frame is set to LOCAL_WORLD_ALIGNED, it returns the Jacobian expressed in the coordinate system of the frame centered on the joint, but aligned with the WORLD axes.
        If reference_frame is set to WORLD, it returns the Jacobian expressed in the coordinate system of the frame associated to the WORLD.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """

def getJointJacobianTimeVariation(model: Model, data: Data, joint_id: int, reference_frame: ReferenceFrame) -> numpy.ndarray:
    """
    getJointJacobianTimeVariation( (Model)model, (Data)data, (int)joint_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the Jacobian time variation of a specific joint expressed in the requested frame provided by the value of reference_frame.You have to call computeJointJacobiansTimeVariation first. This function also computes the full model Jacobian contained in data.J.
        If reference_frame is set to LOCAL, it returns the Jacobian expressed in the local coordinate system of the joint.
        If reference_frame is set to LOCAL_WORLD_ALIGNED, it returns the Jacobian expressed in the coordinate system of the frame centered on the joint, but aligned with the WORLD axes.
        If reference_frame is set to WORLD, it returns the Jacobian expressed in the coordinate system of the frame associated to the WORLD.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """

def getJointVelocityDerivatives(model: Model, data: Data, joint_id: int, reference_frame: ReferenceFrame) -> tuple:
    """
    getJointVelocityDerivatives( (Model)model, (Data)data, (int)joint_id, (ReferenceFrame)reference_frame) -> tuple :
        Computes the partial derivatives of the spatial velocity of a given joint with respect to
        the joint configuration and velocity and returns them as a tuple.
        The partial derivatives can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.
        You must first call computeForwardKinematicsDerivatives before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """

def getKKTContactDynamicMatrixInverse(model: Model, data: Data, constraint_jacobian: numpy.ndarray) -> numpy.ndarray:
    """
    getKKTContactDynamicMatrixInverse( (Model)model, (Data)data, (numpy.ndarray)constraint_jacobian) -> numpy.ndarray :
        Computes the inverse of the constraint matrix [[M Jt], [J 0]].
         forwardDynamics or impulseDynamics must have been called first.
        Note: the constraint Jacobian should be the same that was provided to forwardDynamics or impulseDynamics.
    """

def getPointClassicAccelerationDerivatives(model: Model, data: Data, joint_id: int, placement: SE3, reference_frame: ReferenceFrame) -> tuple:
    """
    getPointClassicAccelerationDerivatives( (Model)model, (Data)data, (int)joint_id, (SE3)placement, (ReferenceFrame)reference_frame) -> tuple :
        Computes the partial derivatives of the classic acceleration of a point given by its placement information w.r.t. the joint frame and returns them as a tuple.
        The partial derivatives can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.
        You must first call computeForwardKinematicsDerivatives before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	placement: relative placement of the point w.r.t. the joint frame
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """

def getPointVelocityDerivatives(model: Model, data: Data, joint_id: int, placement: SE3, reference_frame: ReferenceFrame) -> tuple:
    """
    getPointVelocityDerivatives( (Model)model, (Data)data, (int)joint_id, (SE3)placement, (ReferenceFrame)reference_frame) -> tuple :
        Computes the partial derivatives of the velocity of a point given by its placement information w.r.t. the joint frame and returns them as a tuple.
        The partial derivatives can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.
        You must first call computeForwardKinematicsDerivatives before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	placement: relative placement of the point w.r.t. the joint frame
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """

def getVelocity(model: Model, data: Data, joint_id_: int, reference_frame: ReferenceFrame = pinocchio_pywrap_default.ReferenceFrame.LOCAL) -> Motion:
    """
    getVelocity( (Model)model, (Data)data, (int)joint_id [, (ReferenceFrame)reference_frame=pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL]) -> Motion :
        Returns the spatial velocity of the joint expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v[,a]) should be called first to compute the joint spatial velocity stored in data.v
    """

@typing.overload
def impulseDynamics(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, contact_models: StdVec_RigidConstraintModel, contact_datas_: StdVec_RigidConstraintData, r_coeff: float = 0, prox_settings: ProximalSettings = ProximalSettings(1e-12, 1e-12, 0, 1)) -> numpy.ndarray:
    """
    impulseDynamics( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v_before, (numpy.ndarray)constraint_jacobian [, (float)restitution_coefficient=0 [, (float)damping=0]]) -> numpy.ndarray :
        Solves the impact dynamics problem with contacts, store the result in context::Data::dq_after and return it. The contact impulses are stored in data.impulse_c.
        Note: internally, pinocchio.crba is called.
    """
@typing.overload
def impulseDynamics(model: Model, data: Data, q: numpy.ndarray, v_before: numpy.ndarray, constraint_jacobian_: numpy.ndarray, restitution_coefficient: float = 0, damping: float = 0) -> numpy.ndarray:
    pass
@typing.overload
def impulseDynamics(model: Model, data: Data, v_before: numpy.ndarray, constraint_jacobian_: numpy.ndarray, restitution_coefficient: float = 0, damping: float = 0) -> numpy.ndarray:
    pass

def initConstraintDynamics(model: Model, data: Data, contact_models: StdVec_RigidConstraintModel) -> None:
    """
    initConstraintDynamics( (Model)model, (Data)data, (StdVec_RigidConstraintModel)contact_models) -> None :
        This function allows to allocate the memory before hand for contact dynamics algorithms.
        This allows to avoid online memory allocation when running these algorithms.
    """

def integrate(model: Model, q: numpy.ndarray, v: numpy.ndarray) -> numpy.ndarray:
    """
    integrate( (Model)model, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Integrate the joint configuration vector q with a tangent vector v during one unit time.
        This is the canonical integrator of a Configuration Space composed of Lie groups, such as most robots.
        
        Parameters:
        	model: model of the kinematic tree
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """

def interpolate(model: Model, q1: numpy.ndarray, q2: numpy.ndarray, alpha: float) -> numpy.ndarray:
    """
    interpolate( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2, (float)alpha) -> numpy.ndarray :
        Interpolate between two given joint configuration vectors q1 and q2.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: the initial joint configuration vector (size model.nq)
        	q2: the terminal joint configuration vector (size model.nq)
        	alpha: the interpolation coefficient in [0,1]
        
    """

def isNormalized(model: Model, q_: numpy.ndarray, prec: float = 1e-12) -> bool:
    """
    isNormalized( (Model)model, (numpy.ndarray)q [, (float)prec=1e-12]) -> bool :
        Check whether a configuration vector is normalized within the given precision provided by prec.
        
        Parameters:
        	model: model of the kinematic tree
        	q: a joint configuration vector (size model.nq)
        	prec: requested accuracy for the check
        
    """

def isSameConfiguration(model: Model, q1: numpy.ndarray, q2: numpy.ndarray, prec: float) -> bool:
    """
    isSameConfiguration( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2, (float)prec) -> bool :
        Return true if two configurations are equivalent within the given precision provided by prec.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: a joint configuration vector (size model.nq)
        	q2: a joint configuration vector (size model.nq)
        	prec: requested accuracy for the comparison
        
    """

@typing.overload
def jacobianCenterOfMass(model: Model, data: Data, q_: numpy.ndarray, compute_subtree_coms: bool = True) -> numpy.ndarray:
    """
    jacobianCenterOfMass( (Model)model, (Data)data, (numpy.ndarray)q [, (bool)compute_subtree_coms=True]) -> numpy.ndarray :
        Computes the Jacobian of the center of mass, puts the result in context::Data and return it.
        If compute_subtree_coms is True, the algorithm also computes the center of mass of the subtrees.
    """
@typing.overload
def jacobianCenterOfMass(model: Model, data_: Data, compute_subtree_coms: bool = True) -> numpy.ndarray:
    pass

@typing.overload
def jacobianSubtreeCenterOfMass(model: Model, data: Data, q: numpy.ndarray, subtree_root_joint_id: int) -> numpy.ndarray:
    """
    jacobianSubtreeCenterOfMass( (Model)model, (Data)data, (numpy.ndarray)q, (int)subtree_root_joint_id) -> numpy.ndarray :
        Computes the Jacobian of the CoM of the given subtree (subtree_root_joint_id) expressed in the WORLD frame, according to the given joint configuration.
    """
@typing.overload
def jacobianSubtreeCenterOfMass(model: Model, data: Data, subtree_root_joint_id: int) -> numpy.ndarray:
    pass

def jointBodyRegressor(model: Model, data: Data, joint_id: int) -> numpy.ndarray:
    """
    jointBodyRegressor( (Model)model, (Data)data, (int)joint_id) -> numpy.ndarray :
        Compute the regressor for the dynamic parameters of a rigid body attached to a given joint.
        This algorithm assumes RNEA has been run to compute the acceleration and gravitational effects.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        
    """

def loadReferenceConfigurations(model: Model, srdf_filename_: object, verbose: bool = False) -> None:
    """
    loadReferenceConfigurations( (Model)model, (object)srdf_filename [, (bool)verbose=False]) -> None :
        Retrieve all the reference configurations of a given model from the SRDF file.
        Parameters:
        	model: model of the robot
        	srdf_filename: path to the SRDF file containing the reference configurations
        	verbose: [optional] display to the current terminal some internal information
    """

def loadReferenceConfigurationsFromXML(model: Model, srdf_xml_stream_: str, verbose: bool = False) -> None:
    """
    loadReferenceConfigurationsFromXML( (Model)model, (str)srdf_xml_stream [, (bool)verbose=False]) -> None :
        Retrieve all the reference configurations of a given model from the SRDF file.
        Parameters:
        	model: model of the robot
        	srdf_xml_stream: XML stream containing the SRDF information with the reference configurations
        	verbose: [optional] display to the current terminal some internal information
    """

def loadRotorParameters(model: Model, srdf_filename_: object, verbose: bool = False) -> bool:
    """
    loadRotorParameters( (Model)model, (object)srdf_filename [, (bool)verbose=False]) -> bool :
        Load the rotor parameters of a given model from a SRDF file.
        Results are stored in model.rotorInertia and model.rotorGearRatio.This function also fills the armature of the model.Parameters:
        	model: model of the robot
        	srdf_filename: path to the SRDF file containing the rotor parameters
        	verbose: [optional] display to the current terminal some internal information
    """


@typing.overload
def log3(R: numpy.ndarray) -> numpy.ndarray:
    """
    log3( (numpy.ndarray)R) -> numpy.ndarray :
        Log: SO3 -> so3 is the pseudo-inverse of Exp: so3 -> SO3. Log maps from SO3 -> { v in so3, ||v|| < 2pi }.
    """
@typing.overload
def log3(R: numpy.ndarray, theta: float) -> numpy.ndarray:
    pass
@typing.overload
def log3(R: numpy.ndarray, theta: numpy.ndarray) -> numpy.ndarray:
    pass
@typing.overload
def log3(quat: Quaternion) -> numpy.ndarray:
    pass
@typing.overload
def log3(quat: numpy.ndarray) -> numpy.ndarray:
    pass
@typing.overload
def log3(quat: numpy.ndarray, theta: float) -> numpy.ndarray:
    pass
@typing.overload
def log3(quat: numpy.ndarray, theta: numpy.ndarray) -> numpy.ndarray:
    pass

@typing.overload
def log6(M: SE3) -> Motion:
    """
    log6( (SE3)M) -> Motion :
        Log: SE3 -> se3. Pseudo-inverse of exp from SE3 -> { v,w in se3, ||w|| < 2pi }.
    """
@typing.overload
def log6(homegeneous_matrix: numpy.ndarray) -> Motion:
    pass

def log6_quat(q: numpy.ndarray) -> Motion:
    """
    log6_quat( (numpy.ndarray)q) -> Motion :
        Log: R^3 * S^3 -> se3. Pseudo-inverse of Exp: se3 -> R^3 * S^3,
    """

def neutral(model: Model) -> numpy.ndarray:
    """
    neutral( (Model)model) -> numpy.ndarray :
        Returns the neutral configuration vector associated to the model.
        
        Parameters:
        	model: model of the kinematic tree
        
    """


def nonLinearEffects(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray) -> numpy.ndarray:
    """
    nonLinearEffects( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Compute the Non Linear Effects (coriolis, centrifugal and gravitational effects), store the result in Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """

def normalize(model: Model, q: numpy.ndarray) -> numpy.ndarray:
    """
    normalize( (Model)model, (numpy.ndarray)q) -> numpy.ndarray :
        Returns the configuration normalized.
        For instance, when the configuration vectors contains some quaternion values, it must be required to renormalize these components to keep orthonormal rotation values.
        
        Parameters:
        	model: model of the kinematic tree
        	q: a joint configuration vector to normalize (size model.nq)
        
    """

def printVersion(delimiter: str = '.') -> str:
    """
    printVersion([  (str)delimiter='.']) -> str :
        Returns the current version of Pinocchio as a string.
        The user may specify the delimiter between the different semantic numbers.
    """

@typing.overload
def randomConfiguration(model: Model) -> numpy.ndarray:
    """
    randomConfiguration( (Model)model) -> numpy.ndarray :
        Generate a random configuration in the bounds given by the lower and upper limits contained in model.
        
        Parameters:
        	model: model of the kinematic tree
        
    """
@typing.overload
def randomConfiguration(model: Model, lower_bound: numpy.ndarray, upper_bound: numpy.ndarray) -> numpy.ndarray:
    pass

def removeCollisionPairs(model: Model, geom_model: GeometryModel, srdf_filename_: object, verbose: bool = False) -> None:
    """
    removeCollisionPairs( (Model)model, (GeometryModel)geom_model, (object)srdf_filename [, (bool)verbose=False]) -> None :
        Parse an SRDF file in order to remove some collision pairs for a specific GeometryModel.
        Parameters:
        Parameters:
        	model: model of the robot
        	geom_model: geometry model of the robot
        	srdf_filename: path to the SRDF file containing the collision pairs to remove
        	verbose: [optional] display to the current terminal some internal information
    """

def removeCollisionPairsFromXML(model: Model, geom_model: GeometryModel, srdf_xml_stream_: str, verbose: bool = False) -> None:
    """
    removeCollisionPairsFromXML( (Model)model, (GeometryModel)geom_model, (str)srdf_xml_stream [, (bool)verbose=False]) -> None :
        Parse an SRDF file in order to remove some collision pairs for a specific GeometryModel.
        Parameters:
        Parameters:
        	model: model of the robot
        	geom_model: geometry model of the robot
        	srdf_xml_stream: XML stream containing the SRDF information with the collision pairs to remove
        	verbose: [optional] display to the current terminal some internal information
    """

@typing.overload
def rnea(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, a: numpy.ndarray) -> numpy.ndarray:
    """
    rnea( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> numpy.ndarray :
        Compute the RNEA, store the result in Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        
    """
@typing.overload
def rnea(model: Model, data: Data, q: numpy.ndarray, v: numpy.ndarray, a: numpy.ndarray, fext: StdVec_Force) -> numpy.ndarray:
    pass

def seed(seed_value: int) -> None:
    """
    seed( (int)seed_value) -> None :
        Initialize the pseudo-random number generator with the argument seed_value.
    """

@typing.overload
def sharedMemory() -> bool:
    """
    sharedMemory( (bool)value) -> None :
        Share the memory when converting from Eigen to Numpy.
    """
@typing.overload
def sharedMemory(value: bool) -> None:
    pass

def skew(u: numpy.ndarray) -> numpy.ndarray:
    """
    skew( (numpy.ndarray)u) -> numpy.ndarray :
        Computes the skew representation of a given 3d vector, i.e. the antisymmetric matrix representation of the cross product operator, aka U = [u]x.
        Parameters:
        	u: the input vector of dimension 3
    """

def skewSquare(u: numpy.ndarray, v: numpy.ndarray) -> numpy.ndarray:
    """
    skewSquare( (numpy.ndarray)u, (numpy.ndarray)v) -> numpy.ndarray :
        Computes the skew square representation of two given 3d vectors, i.e. the antisymmetric matrix representation of the chained cross product operator, u x (v x w), where w is another 3d vector.
        Parameters:
        	u: the first input vector of dimension 3
        	v: the second input vector of dimension 3
    """

def squaredDistance(model: Model, q1: numpy.ndarray, q2: numpy.ndarray) -> numpy.ndarray:
    """
    squaredDistance( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2) -> numpy.ndarray :
        Squared distance vector between two joint configuration vectors.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: the initial joint configuration vector (size model.nq)
        	q2: the terminal joint configuration vector (size model.nq)
        
    """

def unSkew(U: numpy.ndarray) -> numpy.ndarray:
    """
    unSkew( (numpy.ndarray)U) -> numpy.ndarray :
        Inverse of skew operator. From a given skew symmetric matrix U (i.e U = -U.T)of dimension 3x3, it extracts the supporting vector, i.e. the entries of U.
        Mathematically speacking, it computes v such that U.dot(x) = cross(u, x).
        Parameters:
        	U: the input skew symmetric matrix of dimension 3x3.
    """

def updateFramePlacement(model: Model, data: Data, frame_id: int) -> SE3:
    """
    updateFramePlacement( (Model)model, (Data)data, (int)frame_id) -> SE3 :
        Computes the placement of the given operational frame (frame_id) according to the current joint placement stored in data, stores the results in data and returns it.
    """

def updateFramePlacements(model: Model, data: Data) -> None:
    """
    updateFramePlacements( (Model)model, (Data)data) -> None :
        Computes the placements of all the operational frames according to the current joint placement stored in dataand puts the results in data.
    """

@typing.overload
def updateGeometryPlacements(model: Model, data: Data, geometry_model: GeometryModel, geometry_data: GeometryData) -> None:
    """
    updateGeometryPlacements( (Model)model, (Data)data, (GeometryModel)geometry_model, (GeometryData)geometry_data, (numpy.ndarray)q) -> None :
        Update the placement of the collision objects according to the current configuration.
        The algorithm also updates the current placement of the joint in Data.
    """
@typing.overload
def updateGeometryPlacements(model: Model, data: Data, geometry_model: GeometryModel, geometry_data: GeometryData, q: numpy.ndarray) -> None:
    pass

def updateGlobalPlacements(model: Model, data: Data) -> None:
    """
    updateGlobalPlacements( (Model)model, (Data)data) -> None :
        Updates the global placements of all joint frames of the kinematic tree and store the results in data according to the relative placements of the joints.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        
    """

ACCELERATION = pinocchio.pinocchio_pywrap_default.KinematicLevel.ACCELERATION
ARG0 = pinocchio.pinocchio_pywrap_default.ArgumentPosition.ARG0
ARG1 = pinocchio.pinocchio_pywrap_default.ArgumentPosition.ARG1
ARG2 = pinocchio.pinocchio_pywrap_default.ArgumentPosition.ARG2
ARG3 = pinocchio.pinocchio_pywrap_default.ArgumentPosition.ARG3
ARG4 = pinocchio.pinocchio_pywrap_default.ArgumentPosition.ARG4
BODY = pinocchio.pinocchio_pywrap_default.FrameType.BODY
COLLISION = pinocchio.pinocchio_pywrap_default.GeometryType.COLLISION
FIXED_JOINT = pinocchio.pinocchio_pywrap_default.FrameType.FIXED_JOINT
JOINT = pinocchio.pinocchio_pywrap_default.FrameType.JOINT
LOCAL = pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL
LOCAL_WORLD_ALIGNED = pinocchio.pinocchio_pywrap_default.ReferenceFrame.LOCAL_WORLD_ALIGNED
OP_FRAME = pinocchio.pinocchio_pywrap_default.FrameType.OP_FRAME
PINOCCHIO_MAJOR_VERSION = 3
PINOCCHIO_MINOR_VERSION = 4
PINOCCHIO_PATCH_VERSION = 0
POSITION = pinocchio.pinocchio_pywrap_default.KinematicLevel.POSITION
SENSOR = pinocchio.pinocchio_pywrap_default.FrameType.SENSOR
VELOCITY = pinocchio.pinocchio_pywrap_default.KinematicLevel.VELOCITY
VISUAL = pinocchio.pinocchio_pywrap_default.GeometryType.VISUAL
WITH_CPPAD = False
WITH_HPP_FCL = False
WITH_HPP_FCL_BINDINGS = False
WITH_OPENMP = False
WITH_SDFORMAT = False
WITH_URDFDOM = True
WORLD = pinocchio.pinocchio_pywrap_default.ReferenceFrame.WORLD
XAxis: numpy.ndarray # value = array([1., 0., 0.])
YAxis: numpy.ndarray # value = array([0., 1., 0.])
ZAxis: numpy.ndarray # value = array([0., 0., 1.])
__raw_version__ = '3.4.0'
__version__ = '3.4.0'
annotations: __future__._Feature # value = _Feature((3, 7, 0, 'beta', 1), None, 16777216)
module_info: tuple # value = ('serialization', <module 'pinocchio.pinocchio_pywrap_default.serialization'>)
submodules: list # value = [('cholesky', <module 'pinocchio.pinocchio_pywrap_default.cholesky'>), ('liegroups', <module 'pinocchio.pinocchio_pywrap_default.liegroups'>), ('linalg', <module 'pinocchio.pinocchio_pywrap_default.linalg'>), ('rpy', <module 'pinocchio.pinocchio_pywrap_default.rpy'>), ('serialization', <module 'pinocchio.pinocchio_pywrap_default.serialization'>)]
import Boost.Python
import __future__
import inspect
import numpy
import pinocchio.pinocchio_pywrap_default
import pinocchio.pinocchio_pywrap_default.cholesky
import pinocchio.pinocchio_pywrap_default.liegroups
import pinocchio.pinocchio_pywrap_default.linalg
import pinocchio.pinocchio_pywrap_default.rpy
import pinocchio.pinocchio_pywrap_default.serialization
import pinocchio_pywrap_default.Convention
import pinocchio_pywrap_default.ReferenceFrame
import sys
_Shape = typing.Tuple[int, ...]
