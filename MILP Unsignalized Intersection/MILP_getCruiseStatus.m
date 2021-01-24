function [p, v] = MILP_getCruiseStatus(pos, speed, step)
p = pos - speed * step;
v = speed;