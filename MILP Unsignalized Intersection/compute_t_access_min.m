function t_access_min = compute_t_access_min(d, v, v_max, a_max)
    % d
    % v
    % v_max
    % a_max
    
    delta_t1 = (v_max - v) / a_max;
    if delta_t1 > (sqrt(v^2 + 2*a_max*d) - v) / a_max
        delta_t1 = (sqrt(v^2 + 2*a_max*d) - v) / a_max;
    end

    delta_t2 = (d / v_max) - (v_max^2 - v^2)/(2*a_max*v_max);
    if delta_t2 < 0
        delta_t2 = 0;
    end    

    t0 = get_param('trial_model', 'SimulationTime');
    t_access_min = t0 + delta_t1 + delta_t2;
end