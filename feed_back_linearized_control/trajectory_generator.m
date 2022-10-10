function [q_t, vd] = trajectory_generator(q, to, tf)
%a(4, 1) = [a0, a1, a2, a3];
%q (4, 1) = [qo, qo_d, qf, qf_d];

syms t;
time = [1, to, to^2, to^3; 0, 1, 2*to, 3*(to^2); 1, tf, tf^2, tf^3; 0, 1, 2*tf, 3*(tf^2)];
a = inv(time)*q;

qt = a(1) + a(2)*t + a(3)*(t^2) + a(4)*(t^3);
qt_d = a(2) + 2*a(3)*t + 3*a(4)*(t^2);
q_t = [qt; qt_d];
vd = 2*a(3) + 6*a(4)*t;

end