function [p, v] = trajectory_planning(pos, vel, t_access)
%     t0 = get_param('trial_model', 'SimulationTime');
%     p = @(t) pos - vel*(t-t0); 
%     v = @(t) vel;

t0 = get_param('trial_model', 'SimulationTime');
delta_t = t_access - t0;
reference = pos / vel;

if delta_t == reference
    v = @(t) vel;
    p = @(t) pos - vel * (t-t0);
    
elseif delta_t < reference  % need to speed up
    a_acc = 2*(pos - vel*delta_t)/(delta_t^2);
    if a_acc <= 2
        a_acc = 2;
    end
    v_cruise = vel + a_acc*delta_t - sqrt(2*a_acc*(0.5*a_acc*delta_t^2 + vel*delta_t - pos));
    t_star = (2*pos + t0*vel + t0*v_cruise - 2*v_cruise*t_access) / (vel - v_cruise);
    
    v = @(t) (vel + a_acc*(t-t0))*(t >= t0 && t < t_star) + (v_cruise)*(t >= t_star);
    p = @(t) (pos - vel*(t-t0) - 0.5*a_acc*((t-t0)^2))*(t >= t0 && t < t_star) + (pos - v_cruise*(t-t0))*(t >= t_star);
    
else % need to slow down (UNDONE: what if need to completely stop ?)
    a_dec = 2*(pos - vel*delta_t)/(delta_t^2);
    if a_dec >= -2
        a_dec = -2;
    end
    v_cruise = vel + a_dec*delta_t + sqrt(2*a_dec*(0.5*a_dec*delta_t^2 + vel*delta_t - pos));
    t_star = (2*pos + t0*vel + t0*v_cruise - 2*v_cruise*t_access) / (vel - v_cruise);
    
    v = @(t) (vel + a_dec*(t-t0))*(t >= t0 && t < t_star) + (v_cruise)*(t >= t_star);
    p = @(t) (pos - vel*(t-t0) - 0.5*a_dec*((t-t0)^2))*(t >= t0 && t < t_star) + (pos - v_cruise*(t-t0))*(t >= t_star);
end

end