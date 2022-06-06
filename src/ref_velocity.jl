function ref_velocity(p)
  if (p >= 5.0)
    return (0.0, 0.0)
  else
    a = -0.5
    j = -0.2
    u = a * p + 0.5 * j * p^2 + 5.0
    return (u, a + j * p)
  end
end

function set_ref_velocity(p)
  vref = zeros(size(p))
  for i in 1:length(vref)
    vr, ar = ref_velocity(p[i])
    vref[i] = vr
  end
  return vref
end
