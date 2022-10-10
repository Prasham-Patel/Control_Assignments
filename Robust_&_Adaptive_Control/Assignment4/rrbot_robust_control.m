% rrbot robust control in gazebo
% Author: Prasham Patel

clear; close; clc;

% Robust control parameters

b = 0.2; % boundry layer value % use b = 0 for sign function
S = [15, 0; 0, 10]; % uncertainity matrix

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

q = diag([2, 3, 1, 1])
Q = eye(4)*q;
 
P = lyap(Acl.', Q);

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

% Variable declared to store values of states
tic;
t = 0;
U = [];
Y = [];
time = [];

while(t < 10)
    t = toc;
% read the joint 
statesjointData = receive(JointStates);
theta1 = statesjointData.Position(1);
theta2 = statesjointData.Position(2);
theta_dot1 = statesjointData.Velocity(1);
theta_dot2 = statesjointData.Velocity(2);

% inspect the "jointData" variable in MATLAB to get familiar with its structure
% design your state feedback controller in the following

% error term
e = [- (pi*t^3)/500 + (3*pi*t^2)/100 - pi + theta1;
- (pi*t^3)/1000 + (3*pi*t^2)/200 - pi/2 + theta2;
       theta_dot1 + (3*pi*t)/50 - (3*pi*t^2)/500;
     theta_dot2 + (3*pi*t)/100 - (3*pi*t^2)/1000]

% % wraping theta1 and theta2
% if (abs(theta1) > 2*pi) 
%     theta1 = mod(theta1, (2*pi));
% end
% 
% if (abs(theta2) > 2*pi) 
%     theta2 = mod(theta2, (2*pi));
% end

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

T(1) =  - (26487*sin(theta1 + theta2))/8000 - (85347*sin(theta1))/8000 - ((27*cos(theta2))/40 + 4719/4000)*(- (3*pi*t^3)/125 + (159*pi*t^2)/500 + (51*pi*t)/125 - Vr1 - (597*pi)/50 + 12*theta1 + 7*theta_dot1) - ((27*cos(theta2))/80 + 1719/8000)*(- (3*pi*t^3)/250 + (159*pi*t^2)/1000 + (51*pi*t)/250 - Vr2 - (597*pi)/100 + 12*theta2 + 7*theta_dot2) - (27*theta_dot2*sin(theta2)*(theta_dot1 + theta_dot2))/80 - (27*theta_dot1*theta_dot2*sin(theta2))/80;
T(2) =    (1719*Vr2)/8000 - (5157*theta2)/2000 - (12033*theta_dot2)/8000 + (1026243*pi)/800000 - (26487*sin(theta1 + theta2))/8000 + (27*theta_dot1^2*sin(theta2))/80 - (87669*pi*t)/2000000 - (273321*pi*t^2)/8000000 + (5157*pi*t^3)/2000000 - ((27*cos(theta2))/80 + 1719/8000)*(- (3*pi*t^3)/125 + (159*pi*t^2)/500 + (51*pi*t)/125 - Vr1 - (597*pi)/50 + 12*theta1 + 7*theta_dot1);

u = [T(1), T(2)];

% updating values of states in the arrays declared earlier
tau1.Data = T(1);
tau2.Data = T(2);
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
% disconnectfromroscore
rosshutdown;
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
