include("ref_velocity.jl")

function vehicle_model!(dx, x, u, damp)
  p, v, delta = x
  vr, ar = ref_velocity(p)

  dx[1] = v
  dx[2] = -damp * v + u
  dx[3] = (ar + damp) * v - u

  if (v < 0.0)
    dx[1] = 0.0
    dx[2] = 0.0
    dx[3] = 0.0
    if (u > 0.0)
      dx[2] = u
      dx[3] = ar * v - u
    end
  end

  return dx
end
