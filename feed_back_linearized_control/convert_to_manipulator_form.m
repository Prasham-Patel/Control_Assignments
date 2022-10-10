
m(1, 1) = (m1*r1^2 + I1 + I2 + (m2*(2*l1^2 + 4*cos(theta2)*l1*r2 + 2*r2^2))/2);
m(1, 2) = (I2 + (m2*(2*r2^2 + 2*l1*cos(theta2)*r2))/2);
m(2, 1) = (I2 + (m2*(2*r2^2 + 2*l1*cos(theta2)*r2))/2);
m(2, 2) = (m2*r2^2 + I2);

 
c(1, 1) = l1*r2*sin(theta2);
c(1, 2) = - (m2*(2*l1*r2*sin(theta2)*(theta_dot1 + theta_dot2)))/2;
c(2, 1) = l1*m2*r2*sin(theta2)*(theta_dot1 + theta_dot2);
c(2, 2) = - l1*m2*r2*theta_dot1*sin(theta2);


G(1, 1) = - g*m2*(r2*sin(theta1 + theta2) + l1*sin(theta1)) - g*m1*r1*sin(theta1);
G(2, 1) = - g*m2*r2*sin(theta1 + theta2);

T = m*[theta_ddot1; theta_ddot2] + c*[theta_dot1; theta_dot2] + G