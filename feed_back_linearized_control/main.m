patel_progassignment1_matlab
convert_to_manipulator_form

syms kp1 kd1 kp2 kd2

to = 0;
tf = 10;
theta1_o = deg2rad(180);
theta1_f = 0;
theta2_o = deg2rad(90);
theta2_f = 0;
theta1_o_d = 0;
theta1_f_d = 0;
theta2_o_d = 0;
theta2_f_d = 0;

q1 = [theta1_o;theta1_o_d;theta1_f;theta1_f_d]
q2 = [theta2_o;theta2_o_d;theta2_f;theta2_f_d]

[q1_t, vd(1)] = trajectory_generator(q1, to, tf)
[q2_t, vd(2)] = trajectory_generator(q2, to, tf)

v1 = -kp1*(theta1 - q1_t(1)) -kd1*(theta_dot1 - q1_t(2)) + vd(1)
v2 = -kp2*(theta2 - q2_t(1)) -kd2*(theta_dot2 - q2_t(2)) + vd(2)

T1 = subs(u1, {theta_ddot1, theta_ddot2}, {v1, v2})
T2 = subs(u2, {theta_ddot1, theta_ddot2}, {v1, v2})

eq1 = u1-T1;
eq2 = u2-T2;

sol = solve([eq1==0, eq2==0], [theta_ddot1, theta_ddot2]);
th_ddot1 = sol.theta_ddot1 ;
th_ddot2 = sol.theta_ddot2 ;

[t, y] = ode45(@ode, [0, 10], [deg2rad(200), deg2rad(125), 0, 0]);

T1 = subs(T1, {l1 l2 r1 r2 m1 m2 I1 I2 g}, {1, 1, 0.45, 0.45, 1, 1, 0.084, 0.084, 9.81})
T2 = subs(T2, {l1 l2 r1 r2 m1 m2 I1 I2 g}, {1, 1, 0.45, 0.45, 1, 1, 0.084, 0.084, 9.81})

th1_des = (pi*t.^3)/500 - (3*pi*t.^2)/100 + pi;
th1dot_des = (3*pi*t.^2)/500 - (3*pi*t)/50;
th2_des = (pi*t.^3)/1000 - (3*pi*t.^2)/200 + pi/2;
th2dot_des = (3*pi*t.^2)/1000 - (3*pi*t)/100;

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


