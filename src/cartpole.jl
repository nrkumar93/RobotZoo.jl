"""
    Cartpole{T}

Canoncial nonlinear system, with a prismatic joint followed by a revolute joint, or a pendulum
attached to a cart, that is only actuated by sliding the cart. It has 4 states and 1 control.

# Constructors
    Cartpole(; kwargs...)

with keyword arguments
* `mc` - mass of the cart, in kg (default = 1)
* `mp` - mass of the pendulum, in kg (default = 0.2)
* `l` - length of the pendulum, in m (default = 0.5)
* `g` - gravity, in m/s² (default = 9.81)
"""
@autodiff struct Cartpole{T} <: ContinuousDynamics 
    mc::T
    mp::T
    l::T
    g::T
    function Cartpole(mc, mp, l, g)
        T = eltype(promote(mc, mp, l, g))
        new{T}(mc, mp, l, g)
    end
end

Cartpole(; mc=1.0, mp=0.2, l=0.5, g=9.81) = Cartpole(mc, mp, l, g)

function dynamics(model::Cartpole, x, u)
    mc = model.mc  # mass of the cart in kg (10)
    mp = model.mp   # mass of the pole (point mass at the end) in kg
    l = model.l   # length of the pole in m
    g = model.g  # gravity m/s^2

    q = x[ @SVector [1,2] ]
    qd = x[ @SVector [3,4] ]

    s = sin(q[2])
    c = cos(q[2])

    H = @SMatrix [mc+mp mp*l*c; mp*l*c mp*l^2]
    C = @SMatrix [0 -mp*qd[2]*l*s; 0 0]
    G = @SVector [0, mp*g*l*s]
    B = @SVector [1, 0]

    qdd = -H\(C*qd + G - B*u[1])
    return [qd; qdd]
end

function dynamics!(model::Cartpole, xdot, x, u)
    xdot .= dynamics(model, x, u)
end

RobotDynamics.state_dim(::Cartpole) = 4
RobotDynamics.control_dim(::Cartpole) = 1

# Base.position(::Cartpole, x::StaticVector) = SA[0,x[1],0]
# RobotDynamics.orientation(::Cartpole, x::StaticVector) = UnitQuaternion(expm(SA[1,0,0]*x[2]))
