% Author: Parasham Patel

syms theta1 theta2 theta_dot1 theta_dot2 theta_ddot1 theta_ddot2;
syms l1 l2 r1 r2 m1 m2 I1 I2 g;

a = I1 + I2 + m1*r1^2 + m2*(l1^2 + r2^2);
b = m2*l1*r2;
d = I2 + m2*r2^2;

Mmat= [a+2*b*cos(theta2), d+b*cos(theta2); d+b*cos(theta2), d];
Cmat= [-b*sin(theta2)*theta_dot2, -b*sin(theta2)*(theta_dot1+theta_dot2); b*sin(theta2)*theta_dot1,0];
Gmat= [-m1*g*r1*sin(theta1)-m2*g*(l1*sin(theta1)+r2*sin(theta1+theta2)); -m2*g*r2*sin(theta1+theta2)];