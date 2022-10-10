% Adaptive control ODE function
% Author: Prasham Patel 

function xd = adaptive_ode(t, z)

% input matrix is (1, 11), T1 and T2 are also intergrated to calculate control inputs later
z = num2cell(z);
[theta1, theta2, theta_dot1, theta_dot2, alpha1, alpha2, alpha3, alpha4, alpha5, T1, T2] = deal(z{:});
alpha = [ alpha1; alpha2; alpha3; alpha4; alpha5];

% Adaptive control parameters
A = [ 0,   0,   1,  0; 
      0,   0,   0,  1; 
      0,   0,   0,  0; 
      0,   0,   0,  0];

B = [0, 0;
     0, 0;
     1, 0;
     0, 1];

l = [-3, -3, -4, -4];

k = place(A, B, l);

Acl = [zeros(2,2), eye(2); -k];

Q = eye(4)*1.2; 

P = lyap(Acl.', Q);

% error term
e = [theta1 - ((pi*t^3)/500 - (3*pi*t^2)/100 + pi);
     theta2 - ((pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2);
     theta_dot1 - ((3*pi*t^2)/500 - (3*pi*t)/50);
     theta_dot2 - ((3*pi*t^2)/1000 - (3*pi*t)/100)];

% gama matrix to get alpha_dot 
J = eye(5)*0.35;  

% wraping theta1 and theta2
if (abs(theta1) > 2*pi) 
    theta1 = mod(theta1, (2*pi));
end

if (abs(theta2) > 2*pi) 
    theta2 = mod(theta2, (2*pi));
end

% calculating control input according to paramertic equation
Y =[ (3*pi*t^3)/125 - (159*pi*t^2)/500 - (51*pi*t)/125 - 12*theta1 - 7*theta_dot1 + (597*pi)/50, - theta2^2*sin(theta2) - cos(theta2)*(- (3*pi*t^3)/50 + (159*pi*t^2)/200 + (51*pi*t)/50 + 24*theta1 + 12*theta2 + 14*theta_dot1 + 7*theta_dot2 - (597*pi)/20) - 2*theta_dot1*theta_dot2*sin(theta2),                              (3*pi*t^3)/250 - (159*pi*t^2)/1000 - (51*pi*t)/250 - 12*theta2 - 7*theta_dot2 + (597*pi)/100, -(981*sin(theta1))/100, -(981*sin(theta1 + theta2))/100;
                                                                                          0,                                                               theta_dot1^2*sin(theta2) - cos(theta2)*(- (3*pi*t^3)/125 + (159*pi*t^2)/500 + (51*pi*t)/125 + 12*theta1 + 7*theta_dot1 - (597*pi)/50), (9*pi*t^3)/250 - (477*pi*t^2)/1000 - (153*pi*t)/250 - 12*theta1 - 12*theta2 - 7*theta_dot1 - 7*theta_dot2 + (1791*pi)/100,                      0, -(981*sin(theta1 + theta2))/100];

T = Y*alpha;                                                                                   

T1 = double(T(1));
T2 = double(T(2));

% Calculating values of states
theta_ddot1 =-(382000*T1 - 382000*T2 + 5433759*sin(theta1) + 171900*theta_dot1^2*sin(theta2) + 171900*theta_dot2^2*sin(theta2) - 600000*T2*cos(theta2) - 2648700*sin(theta1 + theta2)*cos(theta2) + 343800*theta_dot1*theta_dot2*sin(theta2) + 270000*theta_dot1^2*cos(theta2)*sin(theta2))/(270000*cos(theta2)^2 - 491443);
theta_ddot2 = (1146000*T1 - 6292000*T2 - 22717017*sin(theta1 + theta2) + 16301277*sin(theta1) + 2831400*theta_dot1^2*sin(theta2) + 515700*theta_dot2^2*sin(theta2) + 25604100*cos(theta2)*sin(theta1) + 1800000*T1*cos(theta2) - 3600000*T2*cos(theta2) - 7946100*sin(theta1 + theta2)*cos(theta2) + 1031400*theta_dot1*theta_dot2*sin(theta2) + 1620000*theta_dot1^2*cos(theta2)*sin(theta2) + 810000*theta_dot2^2*cos(theta2)*sin(theta2) + 1620000*theta_dot1*theta_dot2*cos(theta2)*sin(theta2))/(3*(270000*cos(theta2)^2 - 491443));

% calculating Y again with theta_ddot1 and theta_ddot2 for updating alpha_dot

Y_update = [theta_ddot1, ...
     cos(theta2)*(2*theta_ddot1 + theta_ddot2) - 2*sin(theta2)*theta_dot1*theta_dot2 - sin(theta2)*theta2^2, ...
     theta_ddot2, ...
     -sin(theta1)*9.81, ...
     -sin(theta1 + theta2)*9.81; ...
     0, ...
     sin(theta2)*theta_dot1^2 + cos(theta2)*theta_ddot1, ...
     theta_ddot1 + theta_ddot2, ... 
     0, ...
     -sin(theta1+theta2)*9.81];

% m_cap matrix to be updated in each loop
m_cap = [alpha(1) + 2*cos(theta2)*alpha(2), cos(theta2)*alpha(2); ...
         cos(theta2)*alpha(2) + alpha(3), alpha(3)];

phi = m_cap\Y_update;

% Calculating alpha_dot
alpha_dot = -J\phi.'*B.'*P*e;  

xd = [theta_dot1; theta_dot2; theta_ddot1; theta_ddot2; alpha_dot(1); alpha_dot(2); alpha_dot(3); alpha_dot(4); alpha_dot(5); T1; T2];
xd = double(xd);
end