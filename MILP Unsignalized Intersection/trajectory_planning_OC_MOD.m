% MODIFIED FROM OPTIMAL CONTROL
% 
% Given the t_access, distance from access point, current speed
% what is the optimal control (optimal acceleration) ?

function [v, p] = trajectory_planning_OC_MOD(v0, vm, tm, p0, delay)
% t0 = current time (the time instance when you want to run this function)
% v0 = speed at t0
% p0 = distance from access point at t0
% vm = velocity of CAV when it enters access area 
% tm = the optimal t_access calculated from MILP

t0 = get_param('trial_model', 'SimulationTime') + delay;
new_v = v0;
new_p = p0 - v0*delay;

%L = 400;
L = new_p;

% b = fixedtm_fixedvm(v0, vm, t0, tm, p0, L);
b = fixedtm_fixedvm(new_v, vm, t0, tm, 0, L);

v = @(t) (v0)*(t<t0) + (1/2 * b(1) * t^2 + b(2) * t + b(3)) * (t0 <= t && t < tm) ...
    + vm * (t >= tm);

p = @(t) (p0 - v0*(t-t0+delay))*(t<t0) + (new_p - 1/6 * b(1) * t^3 - 1/2 * b(2) * t^2 - b(3) * t - b(4)) ...
    * (t0 <= t && t < tm) + (0 - vm * (t - tm)) * (t >= tm);

end