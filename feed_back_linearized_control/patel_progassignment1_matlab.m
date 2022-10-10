syms theta1 theta2 theta_dot1 theta_dot2 theta_ddot1 theta_ddot2;
syms KE PE;
syms l1 l2 r1 r2 m1 m2 I1 I2 g T1 T2;

KE = 0.5*(I1*(theta_dot1)^2 + m1*(r1*theta_dot1)^2) + 0.5*m2*((theta_dot1*l1)^2 + (r2*(theta_dot1 + theta_dot2))^2 + 2*theta_dot1*l1*r2*(theta_dot1+theta_dot2)*cos(theta2)) + 0.5*(I2*(theta_dot1+theta_dot2)^2); 
PE = g*m2*(r2*cos(theta1 + theta2) + l1*cos(theta1)) + g*m1*r1*cos(theta1);
L = KE - PE;

syms dl_dtheta1 dl_dtheta2 dl_dtheta_dot1 dl_dtheta_dot2;

dl_dtheta1 = jacobian(L, theta1);
dl_dtheta2 = jacobian(L, theta2);
dl_dtheta_dot1 = jacobian(L, theta_dot1);
dl_dtheta_dot2 = jacobian(L, theta_dot2);

syms ddl_dtheta_dot1_dt d_dl_dtheta_dot2_dt;
ddl_dtheta_dot1_dt = jacobian(dl_dtheta_dot1, [theta1; theta_dot1])*[theta_dot1; theta_ddot1] + jacobian(dl_dtheta_dot1, [theta2; theta_dot2])*[theta_dot2; theta_ddot2];
ddl_dtheta_dot2_dt = jacobian(dl_dtheta_dot2, [theta1; theta_dot1])*[theta_dot1; theta_ddot1] + jacobian(dl_dtheta_dot2, [theta2; theta_dot2])*[theta_dot2; theta_ddot2];

 u1 = ddl_dtheta_dot1_dt - dl_dtheta1;
 u2 = ddl_dtheta_dot2_dt - dl_dtheta2;
 
% eq1 = u1-T1;
% eq2 = u2-T2;
% sol = solve([eq1==0, eq2==0], [theta_ddot1, theta_ddot2]);
% th_ddot1 = sol.theta_ddot1 ;
% th_ddot2 = sol.theta_ddot2 ;

%[t, y] = ode45(@ode_2link, [0, 10], [pi/6, pi/4, 0, 0]);
%plot(t, y);