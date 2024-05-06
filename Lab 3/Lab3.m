%{
    Names:
    Ritvik Singh (1005834554)
    Arthur Allshire (1005713587)
    Hshmat Sahak (1005903710)
%}
clc; clear
close all

% 3.1
% create DH table (assume thetas are 0.0 for now)
% this DH table is in cm
DH = [0.0, 76.0, 0.0, pi/2;
    0.0, -23.65, 43.23, 0.0;
    0.0, 0.0, 0.0, pi/2;
    0.0, 43.18, 0.0, -pi/2;
    0.0, 0.0, 0.0, pi/2;
    0.0, 20.0, 0.0, 0.0];
% create instance of puma560
myrobot = mypuma560(DH);

% Testing att.m function as follows. You should get
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4)=100*[-1; 3; 3;]/4;
% IK to get q1 (start)
q1 = inverse(H1,myrobot);
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
% IK to get q2 (final)
q2 = inverse(H2,myrobot);
% Should match desired joint torques from attractive potential
tau = att(q1,q2,myrobot);
%%
% 3.2
% Issue the following commands to plot the trajectory of your robot arm 
qref = motionplan(q1,q2,0,10,myrobot,[],0.01);
t=linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q)
%%
% 3.3
setupobstacle
% Test cylinder
q3 = 0.9*q1+0.1*q2;
tau = rep(q3,myrobot,obs{1});
% Test sphere
q = [pi/2 pi 1.2*pi 0 0 0];
% Joint torque controlled by repulsive potential
tau = rep(q,myrobot,obs{6});
%%
% 3.4

setupobstacle
hold on
axis([-100 100 -100 100 0 200])
view(-32,50)
plotobstacle(obs);
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);
hold off

