"""
    DoubleIntegrator2D{N,M}

A simple point mass in `M` dimensions with direct control over acceleration. Has `N` state
and `M` controls.

# Constructor
    DoubleIntegrator2D([D=1])

where `D` is the dimensionality of the space in which the particle moves. For example,
for a particle in 3D space, `D = 3`.
"""
struct DoubleIntegrator2D <: AbstractModel
    pos::SVector{2,Int}
    vel::SVector{2,Int}
    gravity::SVector{2,Float64}
end

function DoubleIntegrator2D(;gravity=zeros(2))
    D=2
    pos = SVector{D}(1:D)
    vel = SVector{D}(D .+ (1:D))
    DoubleIntegrator2D(pos,vel, gravity)
end

RobotDynamics.state_dim(::DoubleIntegrator2D) = 4
RobotDynamics.control_dim(::DoubleIntegrator2D) = 2


# @generated function dynamics(di::DoubleIntegrator2D, x, u)
function dynamics(di::DoubleIntegrator2D, x, u)
# function dynamics(di::DoubleIntegrator2D, x::SVector{4, Float64}, u::SVector{2, Float64})
    # M = 2
    # vel = [:(x[$i]) for i = M+1:2*M]
    # us = [:(u[$i] + di.gravity[$i]) for i = 1:M]
    # :(SVector{$2*M}($(vel...),$(us...)))
    vel = [(x[i]) for i = 3:4]
    us = [(u[i] + di.gravity[i]) for i = 1:2]
    return [vel; us]
end

Base.position(::DoubleIntegrator2D, x) = @SVector [x[1], x[2], 0]
orientation(::DoubleIntegrator2D, x) = UnitQuaternion(I)
