% Author: Prasham Patel

Y = [theta_ddot1, ...
     cos(theta2)*(2*theta_ddot1 + theta_ddot2) - 2*sin(theta2)*theta_dot1*theta_dot2 - sin(theta2)*theta2^2, ...
     theta_ddot2, ...
     -sin(theta1)*g, ...
     -sin(theta1 + theta2)*g; ...
     0, ...
     sin(theta2)*theta_dot1^2 + cos(theta2)*theta_ddot1, ...
     theta_ddot1 + theta_ddot2, ... 
     0, ...
     -sin(theta1+theta2)*g];
 
 alpha = [m2*l1^2 + m1*r1^2 + m2*r2^2 + I1 + I2;
          m2*l1*r2;
          m2*r2^2 + I2;
          m1*r1 + m2*l1;
          m2*r2];