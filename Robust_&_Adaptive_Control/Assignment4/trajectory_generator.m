% Author: Prasham Patel

function [q_t, vd] = trajectory_generator(q, to, tf)

% q is the matrix containing initial and final states
% to and tf are the starting and ending time

syms t;
time = [1, to, to^2, to^3; 0, 1, 2*to, 3*(to^2); 1, tf, tf^2, tf^3; 0, 1, 2*tf, 3*(tf^2)];
a = inv(time)*q;

qt = a(1) + a(2)*t + a(3)*(t^2) + a(4)*(t^3);
qt_d = a(2) + 2*a(3)*t + 3*a(4)*(t^2);

% q_t contains desired position and velocity equation
% vd is the equation for desired acceleration

q_t = [qt; qt_d];
vd = 2*a(3) + 6*a(4)*t;

end