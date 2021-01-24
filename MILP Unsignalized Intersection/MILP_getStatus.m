function [position, speed] = MILP_getStatus(p, v)
t = get_param('trial_model', 'SimulationTime');
speed = v(t);
position = p(t);
end