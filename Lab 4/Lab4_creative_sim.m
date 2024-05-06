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
% Waypoints
z_grid = 45;
p0 = [620 -220 150];
p1 = [620 -220 z_grid];
p2 = [620 220 z_grid];
p3 = [420 -600 150];
p4 = [420 -600 z_grid];
p5 = [420 600 z_grid];

% Desired end effector frame
R = [0 0 1; 0 -1 0;1 0 0];
H0 = [R, p0'; 0 0 0 1];
H1 = [R, p1'; 0 0 0 1];
H2 = [R, p2'; 0 0 0 1];
H3 = [R, p3'; 0 0 0 1];
H4 = [R, p4'; 0 0 0 1];
H5 = [R, p5'; 0 0 0 1];

% Compute desired joint angles 
q_home = [0 pi/2 0 0 pi/2 0];
q0 = inverse_kuka(H0, kuka);
q1 = inverse_kuka(H1, kuka);
q2 = inverse_kuka(H2, kuka);
q3 = inverse_kuka(H3, kuka);
q4 = inverse_kuka(H4, kuka);
q5 = inverse_kuka(H5, kuka);

%%
% Set up plane and 3 cylinders
setup_creative_obstacle;
num_pts = 30;

% Motion plan between waypoints
qref_home_0 = motionplan(q_home,q0,0,10,kuka_forces,creative_obs,0.03);
t=linspace(0,10,num_pts);
q_home_0=ppval(qref_home_0,t)';
fprintf("Finished p0");

qref_0_1 = motionplan(q0,q1,0,10,kuka_forces,creative_obs,0.03);
t=linspace(0,10,num_pts);
q_0_1=ppval(qref_0_1,t)';
fprintf("Finished p1");

qref_1_2 = motionplan(q1,q2,0,10,kuka_forces,creative_obs,0.03);
t=linspace(0,10,num_pts);
q_1_2=ppval(qref_1_2,t)';
fprintf("Finished p2");

qref_2_3 = motionplan(q2,q3,0,10,kuka_forces,creative_obs,0.05);
t=linspace(0,10,num_pts);
q_2_3=ppval(qref_2_3,t)';
fprintf("Finished p3");

qref_3_4 = motionplan(q3,q4,0,10,kuka_forces,creative_obs,0.05);
t=linspace(0,10,num_pts);
q_3_4=ppval(qref_3_4,t)';
fprintf("Finished p3");

qref_4_5 = motionplan(q4,q5,0,10,kuka_forces,creative_obs,0.05);
t=linspace(0,10,num_pts);
q_4_5=ppval(qref_4_5,t)';
fprintf("Finished p3");

% Plot motion by concatenating joint angles from different motion plans
q = [q_home_0; q_0_1; q_1_2; q_2_3; q_3_4; q_4_5];
figure
axis([-1000, 1000, -1000, 1000, -100, 3000])
hold on;
plotobstacle(creative_obs);
plot(kuka,q);
