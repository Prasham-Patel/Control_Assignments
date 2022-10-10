clear; close; clc;

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
kp1 = 30;
kd1 = 10;
kp2 = 30;
kd2 = 5;
tic;
t = 0;
U = [];
Y = [];
time = [];
while(t < 10)
    t = toc;
% read the joint 
statesjointData = receive(JointStates);
theta1 = statesjointData.Position(1)
theta2 = statesjointData.Position(2)
theta_dot1 = statesjointData.Velocity(1);
theta_dot2 = statesjointData.Velocity(2);
% inspect the "jointData" variable in MATLAB to get familiar with its structure
% design your state feedback controller in the following

T1 = - (8829*sin(theta1 + theta2))/2000 - (28449*sin(theta1))/2000 - (theta_dot2*((9*sin(theta2)*(theta_dot1 + theta_dot2))/10 + (9*theta_dot1*sin(theta2))/10))/2 - ((9*cos(theta2))/10 + 1573/1000)*((3*pi)/50 - (3*pi*t)/250 + kd1*(theta_dot1 + (3*pi*t)/50 - (3*pi*t^2)/500) + kp1*(- (pi*t^3)/500 + (3*pi*t^2)/100 - pi + theta1)) - ((9*cos(theta2))/20 + 573/2000)*((3*pi)/100 - (3*pi*t)/500 + kd2*(theta_dot2 + (3*pi*t)/100 - (3*pi*t^2)/1000) + kp2*(- (pi*t^3)/1000 + (3*pi*t^2)/200 - pi/2 + theta2));
T2 = (1719*pi*t)/1000000 - (8829*sin(theta1 + theta2))/2000 - (1719*pi)/200000 - (573*kd2*(theta_dot2 + (3*pi*t)/100 - (3*pi*t^2)/1000))/2000 - (573*kp2*(- (pi*t^3)/1000 + (3*pi*t^2)/200 - pi/2 + theta2))/2000 - ((9*cos(theta2))/20 + 573/2000)*((3*pi)/50 - (3*pi*t)/250 + kd1*(theta_dot1 + (3*pi*t)/50 - (3*pi*t^2)/500) + kp1*(- (pi*t^3)/500 + (3*pi*t^2)/100 - pi + theta1)) + (9*theta_dot1*sin(theta2)*(theta_dot1 + theta_dot2))/20 - (9*theta_dot1*theta_dot2*sin(theta2))/20;
 
u = [T1, T2]
tau1.Data = u(1);
tau2.Data = u(2);
U(length(U)+1, 1) = u(1)
U(length(U), 2) = u(2)
Y(length(Y)+1, :) = [theta1, theta2, theta_dot1, theta_dot2]; 
time(length(time)+1) = t
send(j1_effort,tau1);
send(j2_effort,tau2);
% you can sample data here to plot at the end
end
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
% disconnectfromroscore
rosshutdown;