include("ref_velocity.jl")
include("vehicle.jl")

function vehicle_pid!(dx, x, param, t)
  p, v, delta = x
  damp, kp = param
  vr, ar = ref_velocity(p)

  u = kp * delta + ar * v;

  # input constraint
  if (u > 1.0)
    u = 1.0
  elseif (u < -1.0)
    u = -1.0
  end

  vehicle_model!(dx, x, u, damp)
end
