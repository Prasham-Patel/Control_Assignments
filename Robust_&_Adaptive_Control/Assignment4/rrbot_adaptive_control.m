% Robot Control take-home exam: Adaptive Control 
% Gazebo Simulation
% Author: Prasham Patel

clear; close; clc;

% adaptive control parameters

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

Acl = A-B*k


Q = eye(4)*0.055;

P = lyap(Acl.', Q);

% gama matrix for calculating alpha_dot

J = eye(5)*0.5;

% initial alpha value
alpha = [1.1798; 0.3375; 0.2149; 1.0875; 0.3375];

% ROS Setup
rosinit;

j1_effort = rospublisher('/rrbot/joint1_effort_controller/command');
j2_effort = rospublisher('/rrbot/joint2_effort_controller/command');
JointStates = rossubscriber('/rrbot/joint_states');
tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j2_effort);
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);req.ModelName = 'rrbot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint2'};
req.JointPositions = [deg2rad(200), deg2rad(125)];
resp = call(client,req,'Timeout',3);

tic;

% Variable declared to store values of states
t = 0;
U = [];
Y = [];
time = [];
t_old = 0;
theta_dot1_old = 0;
theta_dot2_old = 0;

while(t < 10)
    t = toc;
% read the joint 
statesjointData = receive(JointStates);
theta1 = statesjointData.Position(1);
theta2 = statesjointData.Position(2);
theta_dot1 = statesjointData.Velocity(1);
theta_dot2 = statesjointData.Velocity(2);

% Calculating theta_ddot1 and theta_ddot2 from previous and current values
% of theta_dot1 and theta_dot2
theta_ddot1 = (theta_dot1 - theta_dot1_old)/(t - t_old)
theta_ddot2 = (theta_dot2 - theta_dot2_old)/(t - t_old)

% inspect the "jointData" variable in MATLAB to get familiar with its structure
% design your state feedback controller in the following

% Wraping theta1 and theta2
if (abs(theta1) > 2*pi) 
    theta1 = mod(theta1, (2*pi));
end

if (abs(theta2) > 2*pi) 
    theta2 = mod(theta2, (2*pi));
end

% error calculation
e = [theta1 - ((pi*t^3)/500 - (3*pi*t^2)/100 + pi);
     theta2 - ((pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2);
     theta_dot1 - ((3*pi*t^2)/500 - (3*pi*t)/50);
     theta_dot2 - ((3*pi*t^2)/1000 - (3*pi*t)/100)];

% calculating m_cap
m_cap = [alpha(1) + 2*cos(theta2)*alpha(2), cos(theta2)*alpha(2); ...
         cos(theta2)*alpha(2) + alpha(3), alpha(3)];

% final control input calculation     
y =[ (3*pi*t^3)/125 - (159*pi*t^2)/500 - (51*pi*t)/125 - 12*theta1 - 7*theta_dot1 + (597*pi)/50, - theta2^2*sin(theta2) - cos(theta2)*(- (3*pi*t^3)/50 + (159*pi*t^2)/200 + (51*pi*t)/50 + 24*theta1 + 12*theta2 + 14*theta_dot1 + 7*theta_dot2 - (597*pi)/20) - 2*theta_dot1*theta_dot2*sin(theta2),                              (3*pi*t^3)/250 - (159*pi*t^2)/1000 - (51*pi*t)/250 - 12*theta2 - 7*theta_dot2 + (597*pi)/100, -(981*sin(theta1))/100, -(981*sin(theta1 + theta2))/100;
                                                                                          0,                                                               theta_dot1^2*sin(theta2) - cos(theta2)*(- (3*pi*t^3)/125 + (159*pi*t^2)/500 + (51*pi*t)/125 + 12*theta1 + 7*theta_dot1 - (597*pi)/50), (9*pi*t^3)/250 - (477*pi*t^2)/1000 - (153*pi*t)/250 - 12*theta1 - 12*theta2 - 7*theta_dot1 - 7*theta_dot2 + (1791*pi)/100,                      0, -(981*sin(theta1 + theta2))/100];

T = y*alpha; 

% calculating Y again with theta_ddot1 and theta_ddot2 for updating alpha_dot

y_update = [theta_ddot1, ...
     cos(theta2)*(2*theta_ddot1 + theta_ddot2) - 2*sin(theta2)*theta_dot1*theta_dot2 - sin(theta2)*theta2^2, ...
     theta_ddot2, ...
     -sin(theta1)*9.81, ...
     -sin(theta1 + theta2)*9.81; ...
     0, ...
     sin(theta2)*theta_dot1^2 + cos(theta2)*theta_ddot1, ...
     theta_ddot1 + theta_ddot2, ... 
     0, ...
     -sin(theta1+theta2)*9.81];

% updating m_cap
phi = m_cap\y_update;

% Calculating alpha_dot
alpha_dot = -J\phi.'*B.'*P*e;

% updating values
alpha = alpha + alpha_dot*(t - t_old);
t_old = t;
theta_dot1_old = theta_dot1;
theta_dot2_old = theta_dot2;

% updating values of states in the arrays declared earlier
u = [T(1), T(2)];
tau1.Data = u(1);
tau2.Data = u(2);
U(length(U)+1, 1) = u(1);
U(length(U), 2) = u(2);
Y(length(Y)+1, :) = [theta1, theta2, theta_dot1, theta_dot2]; 
time(length(time)+1) = t;

send(j1_effort,tau1);
send(j2_effort,tau2);

end
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);

% disconnect from roscore
rosshutdown
U(2, :) = []
Y(2:4, :) = []

% desired trajectory
th1_des = (pi*time.^3)/500 - (3*pi*time.^2)/100 + pi;
th1dot_des = (3*pi*time.^2)/500 - (3*pi*time)/50;
th2_des = (pi*time.^3)/1000 - (3*pi*time.^2)/200 + pi/2;
th2dot_des = (3*pi*time.^2)/1000 - (3*pi*time)/100;


% ploting graphs
figure;
plot(time,Y(:,1));
xlabel('time (sec)');
ylabel('Radian');
hold 'on';
plot(time, th1_des);
legend('theta1', 'theta1 desired')

figure;
plot(time,Y(:,2));
xlabel('time (sec)');
ylabel('Radian');
hold 'on';
plot(time, th2_des);
legend('theta2', 'theta2 desired')

figure;
plot(time,Y(:,3));
xlabel('time (sec)');
ylabel('Radian/sec');
hold 'on';
plot(time, th1dot_des);
legend('theta1dot', 'theta1dot desired')

figure;
plot(time,Y(:,4));
xlabel('time (sec)');
ylabel('Radian/sec');
hold 'on';
plot(time, th2dot_des);
legend('theta2dot', 'theta2dot desired')

figure;
plot(time,U(:,1));
xlabel('time (sec)');
ylabel('Newton-meter');
legend('Tau_link_1')

figure;
plot(time,U(:,2));
xlabel('time (sec)');
ylabel('Newton-meter');
legend('Tau_link_2')
