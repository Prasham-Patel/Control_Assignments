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
req.JointPositions = [deg2rad(30), deg2rad(45)];
resp = call(client,req,'Timeout',3);
k = [71.6780712913934,25.7031512287617,22.4343895605068,8.67869135591330;20.8624193374030,11.9932842891210,6.91842521360113,3.12055792331669];
tic;
t = 0;
U = []
Y = []
time = []
while(t < 10)
    t = toc;
% read the joint 
statesjointData = receive(JointStates);
th1 = statesjointData.Position(1)
th2 = statesjointData.Position(2)
th_dot1 = statesjointData.Velocity(1);
th_dot2 = statesjointData.Velocity(2);
% inspect the "jointData" variableinMATLAB to get familiar with itsstructure
% design your state feedback controllerinthe following
u = -k*[th1; th2; th_dot1; th_dot2];
tau1.Data = u(1);
tau2.Data = u(2);
U(length(U)+1, 1) = u(1)
U(length(U), 2) = u(2)
Y(length(Y)+1, :) = [th1, th2, th_dot1, th_dot2]; 
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