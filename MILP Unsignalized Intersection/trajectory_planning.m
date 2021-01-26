function [p, v] = trajectory_planning(pos, vel, t_access, delay)
new_vel = vel;
new_pos = pos - vel*delay;
t0 = get_param('trial_model', 'SimulationTime') + delay;

%t0 = get_param('trial_model', 'SimulationTime');
delta_t = t_access - t0;
reference = new_pos / new_vel;

if delta_t == reference
    v = @(t) vel;
    p = @(t) pos - vel*(t-t0+delay);
    
elseif delta_t < reference  % need to speed up
    a_acc = 2*(new_pos - new_vel*delta_t)/(delta_t^2);
    if a_acc <= 2
        a_acc = 2;
    end
    v_cruise = new_vel + a_acc*delta_t - sqrt(2*a_acc*(0.5*a_acc*delta_t^2 + new_vel*delta_t - new_pos));
    t_star = ((2*new_pos + t0*new_vel + t0*v_cruise - 2*v_cruise*t_access) / (new_vel - v_cruise));
    
    v = @(t) (vel)*(t < t0) + (new_vel + a_acc*(t-t0))*(t >= t0 && t < t_star) + (v_cruise)*(t >= t_star);
    p = @(t) (pos - vel*(t-t0+delay))*(t < t0) + (new_pos - new_vel*(t-t0) - 0.5*a_acc*((t-t0)^2))*(t >= t0 && t < t_star) + (new_pos - v_cruise*(t-t0))*(t >= t_star);
    
else % need to slow down (v_min = 10 ?)        
    a_dec = 2*(new_pos - new_vel*delta_t)/(delta_t^2);
    if a_dec >= -2
        a_dec = -2;
    end
    
    t_access_max = compute_t_access_max(new_pos, new_vel, 10, a_dec, t0);
    if t_access >= t_access_max
        a_js = -0.5*(new_vel^2)/new_pos;
        v = @(t) (vel)*(t < t0) + (new_vel + a_js*(t-t0))*(t >= t0 && t < t_access) + (15.64)*(t>=t_access);
        p = @(t) (pos - vel*(t-t0+delay))*(t < t0) + (new_pos - new_vel*(t-t0) - 0.5*a_js*(t-t0)^2)*(t>=t0 && t < t_access) + (-15.64*(t-t_access))*(t>=t_access);
    else        
        v_cruise = new_vel + a_dec*delta_t + sqrt(2*a_dec*(0.5*a_dec*delta_t^2 + new_vel*delta_t - new_pos));
        t_star = ((2*new_pos + t0*new_vel + t0*v_cruise - 2*v_cruise*t_access) / (new_vel - v_cruise));
        
        v = @(t) (vel)*(t < t0) + (new_vel + a_dec*(t-t0))*(t >= t0 && t < t_star) + (v_cruise)*(t >= t_star);
        p = @(t) (pos - vel*(t-t0+delay))*(t < t0) + (new_pos - new_vel*(t-t0) - 0.5*a_dec*((t-t0)^2))*(t >= t0 && t < t_star) + (new_pos - v_cruise*(t-t0))*(t >= t_star);
    end
end

end