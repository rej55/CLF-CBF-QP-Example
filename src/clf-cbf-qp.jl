using JuMP
using OSQP

include("ref_velocity.jl")
include("vehicle.jl")

function vehicle_clf_cbf_qp!(dx, x, param, t)
  # Define Lyapunov function and its Lie derivative
  V(vr, v) = (vr - v)^2
  LfV(vr, ar, v, d) = -2.0 * d * v * (v - vr) + 2.0 * (ar + d) * v * (vr - v)
  LgV(vr, v) = 4.0 * (v - vr)

  # Define barrier function and its Lie derivative
  B(v) = v
  LfB(ar, v, d) = -(2.0 * d + ar) * v
  LgB() = 2.0

  # Get value
  p, v, delta = x
  damp, λ, γ, weight_slack = param
  vr, ar = ref_velocity(p)

  # Set optimization problem
  model = Model(OSQP.Optimizer)

  @variable(model, -1.0 <= u <= 1.0)
  @variable(model, δ)
  
  # ES-CLF constraint
  # d/dt V(x) + λV(x) <= δ
  @constraint(model, LgV(vr, v) * u - δ <= -LfV(vr, ar, v, damp) - λ * V(vr, v))
  
  # ECBF constraint
  # d/dt B(x) + γB(x) >= 0
  @constraint(model, LgB() * u >= -LfB(ar, v, damp) - γ * B(v))
  
  @objective(model, Min, (u - ar * v) * (u - ar * v) + weight_slack * δ * δ)
  optimize!(model)
  
  # Get input value
  uopt = JuMP.value(u) + ar * v

  
  vehicle_model!(dx, x, uopt, damp)
end
