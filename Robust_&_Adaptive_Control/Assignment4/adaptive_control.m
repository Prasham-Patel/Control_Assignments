% Robot Control take-home exam: Adaptive Control
% Author: Prasham Patel

% importing equation in paramertic form and manipulator form
manipulator_form
linear_parametric_form

% Trajectory time
to = 0;
tf = 10;

% Trajectory initial and final points
theta1_o = deg2rad(180);
theta1_f = 0;
theta2_o = deg2rad(90);
theta2_f = 0;
theta1_o_d = 0;
theta1_f_d = 0;
theta2_o_d = 0;
theta2_f_d = 0;

% Trajectory Generation
q1 = [theta1_o;theta1_o_d;theta1_f;theta1_f_d]
q2 = [theta2_o;theta2_o_d;theta2_f;theta2_f_d]

[q1_t, vd(1)] = trajectory_generator(q1, to, tf)
[q2_t, vd(2)] = trajectory_generator(q2, to, tf)

% Adaptive control Parameters
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

Q = eye(4);
 
P = lyap(Acl.', Q);

% error term
e = [theta1 - q1_t(1);
    theta2 - q2_t(1);
     theta_dot1 - q1_t(2);
     theta_dot2 - q2_t(2)];
 
% M matrix formed with initial parameter values
m_cap = subs(Mmat, {l1 l2 r1 r2 m1 m2 I1 I2 g}, {1, 1, 0.45, 0.45, 0.75, 0.75, 0.063, 0.063, 9.81})

% Virtual input for feedback linearization
V = [vd(1) - k(1, 1)*e(1) - k(1, 3)*e(3);
    vd(2) - k(2, 2)*e(2) - k(2, 4)*e(4) ];

% Y matrix formed by substituting in virtual input
Y = subs(Y, {theta_ddot1 theta_ddot2 g}, {V(1) V(2) 9.81})

% phi matrix
phi = m_cap*Y

% J matrix for calculating alpha_dot
j = [1.02 1.01 1.01 0.99 1.01] 

J = eye(5)*diag(j)

alpha_dot = -J\phi.'*B.'*P*e

% initialize alpha with initial parameter values
alpha_init = subs(alpha, {l1 l2 r1 r2 m1 m2 I1 I2 g}, {1, 1, 0.45, 0.45, 0.75, 0.75, 0.063, 0.063, 9.81})
alpha_init = double(alpha_init)

% U matrix formed with the "actual" values for mass and inertia used to upadte state values
syms T1 T2 

U = Mmat*[theta_ddot1; theta_ddot2] + Cmat*[theta_dot1; theta_dot2] + Gmat; 
U = subs(U, {l1 l2 r1 r2 m1 m2 I1 I2 g}, {1, 1, 0.45, 0.45, 1, 1, 0.084, 0.084, 9.81})

eq = U - [T1; T2]

sol = solve(eq==0, [theta_ddot1, theta_ddot2])
U_th_ddot1 = sol.theta_ddot1 
U_th_ddot2 = sol.theta_ddot2 

% ODE function
[t, y] = ode45(@adaptive_ode, [0, 10], [deg2rad(200), deg2rad(125), 0, 0, alpha_init(1), alpha_init(2), alpha_init(3), alpha_init(4), alpha_init(5), 0, 0]);

% Get control input by differentiating the output from the ODE
tau = zeros(1, 2);
for i = 2:length(t)
    tau(length(tau)+1, 1) = (y(i, 10) - y(i-1, 10))/(t(i) - t(i-1))
    tau(length(tau), 2) = (y(i, 11) - y(i-1, 11))/(t(i) - t(i-1))
end
tau(1, :) = [];

% desired trajectory
th1_des = (pi*t.^3)/500 - (3*pi*t.^2)/100 + pi;
th1dot_des = (3*pi*t.^2)/500 - (3*pi*t)/50;
th2_des = (pi*t.^3)/1000 - (3*pi*t.^2)/200 + pi/2;
th2dot_des = (3*pi*t.^2)/1000 - (3*pi*t)/100;


% ploting graphs
figure;
plot(t,y(:,1));
xlabel('time (sec)');
ylabel('Radian');
hold 'on';
plot(t, th1_des);
legend('theta1', 'theta1 desired')

figure;
plot(t,y(:,2));
xlabel('time (sec)');
ylabel('Radian');
hold 'on';
plot(t, th2_des);
legend('theta2', 'theta2 desired')

figure;
plot(t,y(:,3));
xlabel('time (sec)');
ylabel('Radian/sec');
hold 'on';
plot(t, th1dot_des);
legend('theta1dot', 'theta1dot desired')

figure;
plot(t,y(:,4));
xlabel('time (sec)');
ylabel('Radian/sec');
hold 'on';
plot(t, th2dot_des);
legend('theta2dot', 'theta2dot desired')

figure;
plot(t,tau(:,1));
xlabel('time (sec)');
ylabel('Newton-meter');
legend('Tau_link_1')

figure;
plot(t,tau(:,2));
xlabel('time (sec)');
ylabel('Newton-meter');
legend('Tau_link_2')