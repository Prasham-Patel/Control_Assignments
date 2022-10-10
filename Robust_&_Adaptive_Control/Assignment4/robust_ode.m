% Robust control ODE function
% Author: Prasham Patel

function xd = robust_ode(t, z)

b = 0.06; % boundry layer value % use b = 0 for sign function
S = [10, 0;0, 10]; % unceratinity matrix 

% Robust Control Parameters
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

Q = eye(4)*8;
 
P = lyap(Acl.', Q);

% input matrix is (1, 6), Tau(1) and Tau(2) are also intergrated to calculate control inputs later
z = num2cell(z);
[theta1, theta2, theta_dot1, theta_dot2, Tau(1), Tau(2)] = deal(z{:}); 

% error term
e =[ - (pi*t^3)/500 + (3*pi*t^2)/100 + theta1 - pi;
 - (pi*t^3)/1000 + (3*pi*t^2)/200 + theta2 - pi/2;
        theta_dot1 + (3*pi*t)/50 - (3*pi*t^2)/500;
      theta_dot2 + (3*pi*t)/100 - (3*pi*t^2)/1000;];

% wraping theta1 and theta2
if (abs(theta1) > 2*pi) 
    theta1 = mod(theta1, (2*pi));
end

if (abs(theta2) > 2*pi) 
    theta2 = mod(theta2, (2*pi));
end

% Calculating robust control input term 
cap = e.'*P*B;

if b>0
    if norm(cap)>b
        Vr = -(cap/(norm(cap)))*S;
    else
        Vr = -(cap/b)*S;
    end
else
    if norm(cap)~=0
        Vr = -(cap/(norm(cap)))*S;
    else
        Vr = 0;
    end
end

Vr1 = Vr(1);
Vr2 = Vr(2);

% calculating value of control input

Tau(1) =  - (26487*sin(theta1 + theta2))/8000 - (85347*sin(theta1))/8000 - ((27*cos(theta2))/40 + 4719/4000)*(- (3*pi*t^3)/125 + (159*pi*t^2)/500 + (51*pi*t)/125 - Vr1 - (597*pi)/50 + 12*theta1 + 7*theta_dot1) - ((27*cos(theta2))/80 + 1719/8000)*(- (3*pi*t^3)/250 + (159*pi*t^2)/1000 + (51*pi*t)/250 - Vr2 - (597*pi)/100 + 12*theta2 + 7*theta_dot2) - (27*theta_dot2*sin(theta2)*(theta_dot1 + theta_dot2))/80 - (27*theta_dot1*theta_dot2*sin(theta2))/80;
Tau(2) =    (1719*Vr2)/8000 - (5157*theta2)/2000 - (12033*theta_dot2)/8000 + (1026243*pi)/800000 - (26487*sin(theta1 + theta2))/8000 + (27*theta_dot1^2*sin(theta2))/80 - (87669*pi*t)/2000000 - (273321*pi*t^2)/8000000 + (5157*pi*t^3)/2000000 - ((27*cos(theta2))/80 + 1719/8000)*(- (3*pi*t^3)/125 + (159*pi*t^2)/500 + (51*pi*t)/125 - Vr1 - (597*pi)/50 + 12*theta1 + 7*theta_dot1);

% calculating value of states (eq = U - T)
theta_ddot1 = -(3*(245721500*Vr1 - 2948658000*theta1 - 1720050500*theta_dot1 + 2933914710*pi + 905626500*sin(theta1) + 1620000000*theta1*cos(theta2)^2 + 945000000*theta_dot1*cos(theta2)^2 + 28650000*theta_dot1^2*sin(theta2) + 28650000*theta_dot2^2*sin(theta2) - 100254372*pi*t - 78139437*pi*t^2 + 5897316*pi*t^3 - 1611900000*pi*cos(theta2)^2 - 135000000*Vr1*cos(theta2)^2 - 441450000*sin(theta1 + theta2)*cos(theta2) + 57300000*theta_dot1*theta_dot2*sin(theta2) + 55080000*pi*t*cos(theta2)^2 + 42930000*pi*t^2*cos(theta2)^2 - 3240000*pi*t^3*cos(theta2)^2 + 45000000*theta_dot1^2*cos(theta2)*sin(theta2)))/(2000*(270000*cos(theta2)^2 - 491443));
theta_ddot2 =(3*(5897316000*theta2 - 491443000*Vr2 + 3440101000*theta_dot2 - 2933914710*pi - 2524113000*sin(theta1 + theta2) + 1811253000*sin(theta1) - 3240000000*theta2*cos(theta2)^2 - 1890000000*theta_dot2*cos(theta2)^2 + 314600000*theta_dot1^2*sin(theta2) + 57300000*theta_dot2^2*sin(theta2) + 2844900000*cos(theta2)*sin(theta1) + 100254372*pi*t + 78139437*pi*t^2 - 5897316*pi*t^3 + 1611900000*pi*cos(theta2)^2 + 270000000*Vr2*cos(theta2)^2 - 882900000*sin(theta1 + theta2)*cos(theta2) + 114600000*theta_dot1*theta_dot2*sin(theta2) - 55080000*pi*t*cos(theta2)^2 - 42930000*pi*t^2*cos(theta2)^2 + 3240000*pi*t^3*cos(theta2)^2 + 180000000*theta_dot1^2*cos(theta2)*sin(theta2) + 90000000*theta_dot2^2*cos(theta2)*sin(theta2) + 180000000*theta_dot1*theta_dot2*cos(theta2)*sin(theta2)))/(4000*(270000*cos(theta2)^2 - 491443));

xd = [theta_dot1; theta_dot2; theta_ddot1; theta_ddot2; Tau(1); Tau(2)];

end