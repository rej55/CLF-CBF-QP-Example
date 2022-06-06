using DifferentialEquations
using Plots

include("src/ref_velocity.jl")
include("src/clf-cbf-qp.jl")
include("src/p-control.jl")

# Set common parameter
damp = 0.01

# Set initial values
p0 = -5.0
v0 = 6.0
vr0, ar0 = ref_velocity(p0)

x0 = [p0, v0, vr0 - v0]
tspan = (0.0, 10.0)

# Simulate CLF-CBF-QP
λ = 1000 # Factor for ES-CLF
γ = 1 # Factor for ECBF
w = 10^6 # Weight for slack variable
param1 = [damp, λ, γ, w]
prob1 = ODEProblem(vehicle_clf_cbf_qp!, x0, tspan, param1)
sol1 = solve(prob1, saveat=0.1)

# Simulate FF + P control
kp = 10
param2 = [damp, kp]
prob2 = ODEProblem(vehicle_pid!, x0, tspan, param2)
sol2 = solve(prob2, saveat=0.1)

# Plot results
p1 = sol1[1, :]
p2 = sol2[1, :]

v1 = sol1[2, :]
v2 = sol2[2, :]

vref1 = set_ref_velocity(p1)
vref2 = set_ref_velocity(p2)

pt1 = plot(sol1.t, p1, linewidth=2.0, xlabel="time [s]", ylabel="position [m]", label="position", title="CLF-CBF-QP")
pt2 = plot(sol2.t, p2, linewidth=2.0, xlabel="time [s]", ylabel="position [m]", label="position", title="P + FF")
vt1 = plot(sol1.t, [v1, vref1], linewidth=2.0, xlabel="time [s]", ylabel="velocity [m/s]", label=["actual" "reference"])
vt2 = plot(sol2.t, [v2, vref2], linewidth=2.0, xlabel="time [s]", ylabel="velocity [m/s]", label=["actual" "reference"])
vp1 = plot(p1, [v1, vref1], linewidth=2.0, xlabel="position [m]", ylabel="velocity [m/s]", label=["actual" "reference"])
vp2 = plot(p2, [v2, vref2], linewidth=2.0, xlabel="position [m]", ylabel="velocity [m/s]", label=["actual" "reference"])

plot(pt1, pt2, vt1, vt2, vp1, vp2, layout=(3,2))
