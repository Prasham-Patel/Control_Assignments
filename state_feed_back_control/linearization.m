patel_progassignment1_matlab;
T = [T1, T2];
x = [theta1; theta2; theta_dot1; theta_dot2];
xd = [theta_dot1; theta_dot2; theta_ddot1; theta_ddot2];

% Linearization of A

A = jacobian(xd, x);
A = subs(A, {l1 l2 r1 r2 m1 m2 I1 I2 g}, {1, 1, 0.45, 0.45, 1, 1, 0.084, 0.084, 9.81})

% Linearization of B

B = jacobian(xd, T)
B = subs(B, {l1 l2 r1 r2 m1 m2 I1 I2 g}, {1, 1, 0.45, 0.45, 1, 1, 0.084, 0.084, 9.81})


% Finding Eigen Values to check Stability

A1 = subs(A, {theta1, theta2, theta_dot1, theta_dot2}, {0, 0, 0, 0});
A1 = double(A1);
B1 = subs(B, {theta1, theta2, theta_dot1, theta_dot2}, {0, 0, 0, 0});
B1 = double(B1);
e1 = eig(A1);

A2 = subs(A, {theta1, theta2, theta_dot1, theta_dot2}, {pi, pi, 0, 0});
A2 = double(A2);
B2 = subs(B, {theta1, theta2, theta_dot1, theta_dot2}, {0, 0, 0, 0});
B2 = double(B2);
e2 = eig(A2);

A3 = subs(A, {theta1, theta2, theta_dot1, theta_dot2}, {pi, 0, 0, 0});
A3 = double(A3);
B3 = subs(B, {theta1, theta2, theta_dot1, theta_dot2}, {0, 0, 0, 0});
B3 = double(B3);
e3 = eig(A3);

A4 = subs(A, {theta1, theta2, theta_dot1, theta_dot2}, {0, pi, 0, 0});
A4 = double(A4);
B4 = subs(B, {theta1, theta2, theta_dot1, theta_dot2}, {0, 0, 0, 0});
B4 = double(B4);
e4 = eig(A4);

% Controlability for upward position

Co = ctrb(A1, B1);
rank(Co)
