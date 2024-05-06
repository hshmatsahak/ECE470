%% 
% Init DH table and DH_forces table
DH = [0.0, 400, 25, pi/2;
    0.0, 0.0, 315, 0.0;
    0.0, 0.0, 35, pi/2;
    0.0, 365, 0.0, -pi/2;
    0.0, 0.0, 0.0, pi/2;
    0.0, 161.44, -156, 0.0];
DH_forces = [0.0, 400, 25, pi/2;
    0.0, 0.0, 315, 0.0;
    0.0, 0.0, 35, pi/2;
    0.0, 365, 0.0, -pi/2;
    0.0, 0.0, 0.0, pi/2;
    0.0, 161.44, 0.0, 0.0];
% create instance of kuka
kuka = mykuka(DH);
kuka_forces = mykuka(DH_forces);
%%
% Create desired end effector frames
z_grid = 45;
p0 = [370 -440 150];
p1 = [370 -440 z_grid];
p2 = [750 -220 225];
p3 = [620 350 225];
R = [0 0 1; 0 -1 0;1 0 0];
H0 = [R, p0'; 0 0 0 1];
H1 = [R, p1'; 0 0 0 1];
H2 = [R, p2'; 0 0 0 1];
H3 = [R, p3'; 0 0 0 1];

% Compute desired joint angles
q_home = [0 pi/2 0 0 pi/2 0];
q0 = inverse_kuka(H0, kuka);
q1 = inverse_kuka(H1, kuka);
q2 = inverse_kuka(H2, kuka);
q3 = inverse_kuka(H3, kuka);
%%
% Set up plane and 2 cylinders
setupobstacle;
num_pts = 30;

% Motion planning between waypoints
qref_home_0 = motionplan(q_home,q0,0,10,kuka_forces,obs,0.03);
t=linspace(0,10,num_pts);
q_home_0=ppval(qref_home_0,t)';

qref_0_1 = motionplan(q0,q1,0,10,kuka_forces,obs,0.03);
t=linspace(0,10,num_pts);
q_0_1=ppval(qref_0_1,t)';

qref_1_2 = motionplan(q1,q2,0,10,kuka_forces,obs,0.03);
t=linspace(0,10,num_pts);
q_1_2=ppval(qref_1_2,t)';

qref_2_3 = motionplan(q2,q3,0,10,kuka_forces,obs,0.03);
t=linspace(0,10,num_pts);
q_2_3=ppval(qref_2_3,t)';

% Initialize to home, gripper open
setHome(0.04);
setGripper(0);
% Move above cube
for i=1:num_pts
    setAngles(q_home_0(i, :), 0.02);
end
% Move down and pick up cube
setAngles(q1, 0.01);
setGripper(1);
% Move to p2
for i=1:num_pts
    setAngles(q_1_2(i, :), 0.02);
end
% Move to p3
for i=1:num_pts
    setAngles(q_2_3(i, :), 0.02);
end
% drop to basket
setGripper(0);
