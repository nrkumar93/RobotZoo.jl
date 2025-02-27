module RobotZoo

using StaticArrays
using RobotDynamics
using Rotations
using LinearAlgebra
using ForwardDiff, FiniteDiff
const RD = RobotDynamics

using RobotDynamics: ContinuousDynamics, RigidBody, LieGroupModel, @autodiff
import RobotDynamics: dynamics!, dynamics, jacobian!, forces, moments, wrenches, inertia, inertia_inv, orientation
import RobotDynamics: state_dim, control_dim

include("acrobot.jl")
include("car.jl")
include("cartpole.jl")
include("double_integrator.jl")
include("pendulum.jl")
include("quadrotor.jl")
include("planar_quad.jl")
include("yak_plane.jl")
include("satellite.jl")
include("freebody.jl")
include("rocket.jl")

include("random_linear.jl")
include("linearmodel.jl")

include("LinearModels.jl")

end # module
