import casadi.casadi as cs
import opengen as og
import matplotlib.pyplot as plt
import numpy as np
from time import time

# Build parametric optimizer
# ------------------------------------
# nu: input number
# nx: state number + refstate number
# N: predict number
# ts: discrete time
(nu, nx, N, ts) = (3, 3 + 3 + 3, 50, 0.05)
# init parameter used in cost function
(q, qtheta, r, qN, qthetaN) = (10, 0.1, 1, 200, 2)

(wv, hv, wh, hh, angle) = (0.25555, 0.092, 0.27255, 0.28745, 135)

(Ixx, Ixy, Ixz) = (2.464, 0.001, -0.001)
(Iyx, Iyy, Iyz) = (0.001, 2.243, 0.000)
(Izx, Izy, Izz) = (-0.001, 0.000, 3.874)

(Ixx_inverse, Ixy_inverse, Ixz_inverse) = (0.4058, -0.002, 0.0001)
(Iyx_inverse, Iyy_inverse, Iyz_inverse) = (-0.0002, 0.4458, 0.000)
(Izx_inverse, Izy_inverse, Izz_inverse) = (0.0001, 0.000, 0.2581)

# state and input init in casadi
u = cs.SX.sym('u', nu*N)
x = cs.SX.sym('z0', nx)

(pitch_state, yaw_state, roll_state) = (x[0], x[1], x[2])
(pitch_velocity, yaw_velocity, roll_velocity) = (x[3], x[4], x[5])
(pitch_ref, yaw_ref, roll_ref) = (x[6], x[7], x[8])

def dynamics_ct(x, u):
    dpitch = x[3] # pitch
    dyaw = x[4] # yaw
    droll = x[5] # roll
    Ivp = Ixx * dpitch + Ixy * dyaw + Ixz * droll
    Ivy = Iyx * dpitch + Iyy * dyaw + Iyz * droll
    Ivr = Izx * dpitch + Izy * dyaw + Izz * droll
    vIv1 = dyaw * Ivr - droll * Ivy
    vIv2 = droll * Ivp - dpitch * Ivr
    vIv3 = dpitch * Ivy - dyaw * Ivp
    sum1 = u[0] # - vIv1
    sum2 = u[1] # - vIv2
    sum3 = u[2] # - vIv3
    ddpitch = Ixx_inverse * sum1 + Ixy_inverse * sum2 + Ixz_inverse * sum3
    ddyaw = Iyx_inverse * sum1 + Iyy_inverse * sum2 + Iyz_inverse * sum3
    ddroll = Izx_inverse * sum1 + Izy_inverse * sum2 + Izz_inverse * sum3
    return [dpitch, dyaw, droll, ddpitch, ddyaw, ddroll]

def dynamics_dt(x, u):
    (dpitch, dyaw, droll, ddpitch, ddyaw, ddroll) = dynamics_ct(x, u)
    output = [x[0] + ts * dpitch, x[1] + ts * dyaw, x[2] + ts * droll, 
     x[3] + ts * ddpitch, x[4] + ts * ddyaw, x[5] + ts * ddroll,
     x[6], x[7], x[8]]
    return output

def stage_cost(x, u):
    cost = q * ((x[0] - x[6])**2 + (x[1] - x[7])**2 + (x[2] - x[8])**2) + r * cs.dot(u, u)
    return cost

def terminal_cost(x):
    cost = qN * ((x[0] - x[6])**2 + (x[1] - x[7])**2 + (x[2] - x[8])**2)
    return cost

x_t = x
total_cost = 0
F1 = []
F2 = []
F3 = []
for t in range(0, nu*N, nu):
    u_t = u[t:t+3]
    total_cost += stage_cost(x_t, u_t)  # update cost
    x_t = dynamics_dt(x_t, u_t)         # update state
    # F1 = cs.vertcat(F1, x_t[3])
    # F2 = cs.vertcat(F2, x_t[4])
    # F3 = cs.vertcat(F3, x_t[5])

total_cost += terminal_cost(x_t)  # terminal cost

umin = [-204.44, -73.6, -308.355] * (N)
umax = [204.44, 73.6, 308.355] * (N)
bounds = og.constraints.Rectangle(umin, umax)
# C_bounds = og.constraints.BallInf(None, 100)

problem = og.builder.Problem(u, x, total_cost)  \
            .with_constraints(bounds) \
            # .with_aug_lagrangian_constraints(F1, C_bounds) \
            # .with_aug_lagrangian_constraints(F2, C_bounds) \
            # .with_aug_lagrangian_constraints(F3, C_bounds)

build_config = og.config.BuildConfiguration()  \
    .with_build_directory("rov_optimizers_test1")      \
    .with_build_mode("release")    \
    .with_tcp_interface_config()

meta = og.config.OptimizerMeta().with_optimizer_name("navigation1")

solver_config = og.config.SolverConfiguration().with_tolerance(1e-5)

builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config)
builder.build()
