"""
    DoubleIntegrator1D{N,M}

A simple point mass in `M` dimensions with direct control over acceleration. Has `N` state
and `M` controls.

# Constructor
    DoubleIntegrator1D([D=1])

where `D` is the dimensionality of the space in which the particle moves. For example,
for a particle in 3D space, `D = 3`.
"""
struct DoubleIntegrator1D <: AbstractModel
    pos::SVector{1,Int}
    vel::SVector{1,Int}
    gravity::SVector{1,Float64}
end

function DoubleIntegrator1D(;gravity=zeros(1))
    D=1
    pos = SVector{D}(1:D)
    vel = SVector{D}(D .+ (1:D))
    DoubleIntegrator1D(pos,vel, gravity)
end

RobotDynamics.state_dim(::DoubleIntegrator1D) = 2
RobotDynamics.control_dim(::DoubleIntegrator1D) = 1


# @generated function dynamics(di::DoubleIntegrator1D, x, u)
function dynamics(di::DoubleIntegrator1D, x, u)
# function dynamics(di::DoubleIntegrator1D, x::SVector{4, Float64}, u::SVector{2, Float64})
    # M = 2
    # vel = [:(x[$i]) for i = M+1:2*M]
    # us = [:(u[$i] + di.gravity[$i]) for i = 1:M]
    # :(SVector{$2*M}($(vel...),$(us...)))
    M = 1
    vel = [(x[i]) for i = M+1:2*M]
    us = [(u[i] + di.gravity[i]) for i = 1:M]
    return [vel; us]
end

# Base.position(::DoubleIntegrator1D, x) = @SVector [x[1], x[2], 0]
# orientation(::DoubleIntegrator1D, x) = UnitQuaternion(I)
