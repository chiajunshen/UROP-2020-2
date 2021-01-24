function t_access_max = compute_t_access_max(d, v, v_min, a_dec)

lol = v^2 + 2*a_dec*d;
if lol < 0
    lol = 0;
end

delta_t1 = (v_min - v) / a_dec;
if delta_t1 > (sqrt(lol) - v) / a_dec
    delta_t1 = (sqrt(lol) - v) / a_dec;
end

delta_t2 = (d / v_min) - (v_min^2 - v^2)/(2*a_dec*v_min);
if delta_t2 < 0
    delta_t2 = 0;
end

t0 = get_param('trial_model', 'SimulationTime');
t_access_max = t0 + delta_t1 + delta_t2;
end