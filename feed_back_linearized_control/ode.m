function xd = ode(t, z)

kp1 = 45;
kp2 = 45;
kd1 = 10;
kd2 = 10;

T = zeros(2,1);
z = num2cell(z);
[theta1, theta2, theta_dot1, theta_dot2] = deal(z{:});

if (abs(theta1) > 2*pi) 
    theta1 = mod(theta1, (2*pi));
end

if (abs(theta2) > 2*pi) 
    theta2 = mod(theta2, (2*pi));
end

theta_ddot1 =  pi*kp1 - (3*pi)/50 + (3*pi*t)/250 - kd1*theta_dot1 - kp1*theta1 - (3*pi*kd1*t)/50 + (3*pi*kd1*t^2)/500 - (3*pi*kp1*t^2)/100 + (pi*kp1*t^3)/500;

theta_ddot2 = (pi*kp2)/2 - (3*pi)/100 + (3*pi*t)/500 - kd2*theta_dot2 - kp2*theta2 - (3*pi*kd2*t)/100 + (3*pi*kd2*t^2)/1000 - (3*pi*kp2*t^2)/200 + (pi*kp2*t^3)/1000;
 
xd = [theta_dot1; theta_dot2; theta_ddot1; theta_ddot2];


end