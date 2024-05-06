%{
    Names:
    Ritvik Singh (1005834554)
    Arthur Allshire (1005713587)
    Hshmat Sahak (1005903710)
%}

% Pre-lab
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
% Set up obstacles
setupobstacle_lab4prep;
tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6],kuka_forces, prepobs{1});
% Verify correctness
assert(norm(tau - [0.1795 0.9540 0.2353 -0.0344 -0.0344 0.0000]) < 1e-4);
% Visually verify correctness of prelab motion plan
p1 = [620 375 50];
p2 = [620 -375 50];
R = [0 0 1;0 -1 0;1 0 0];
H1=[R p1';zeros(1,3) 1];
H2=[R p2';zeros(1,3) 1];
q1 = inverse_kuka(H1, kuka);
q2 = inverse_kuka(H2, kuka);
qref = motionplan(q1, q2,0,10,kuka_forces,prepobs,0.01);
t=linspace(0,10,300);
q=ppval(qref,t)';
figure
axis([-1000, 1000, -1000, 1000, -100, 3000])
hold on;
plotobstacle(prepobs);
plot(kuka,q);
%%
% Desired points
z_grid = 45;
p0 = [370 -440 150];
p1 = [370 -440 z_grid];
p2 = [750 -220 225];
p3 = [620 350 225];

% Desired end effector homogenous coordinates
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

% Set up plane and 2 cylinders
setupobstacle;

% Motion plan between waypoints
qref_home_0 = motionplan(q_home,q0,0,10,kuka_forces,obs,0.03);
t=linspace(0,10,100);
q_home_0=ppval(qref_home_0,t)';

qref_0_1 = motionplan(q0,q1,0,10,kuka_forces,obs,0.03);
t=linspace(0,10,100);
q_0_1=ppval(qref_0_1,t)';

qref_1_2 = motionplan(q1,q2,0,10,kuka_forces,obs,0.03);
t=linspace(0,10,100);
q_1_2=ppval(qref_1_2,t)';

qref_2_3 = motionplan(q2,q3,0,10,kuka_forces,obs,0.03);
t=linspace(0,10,100);
q_2_3=ppval(qref_2_3,t)';

% Plot motion by concatenating joint angles from different motion plans
q = [q_home_0; q_0_1; q_1_2; q_2_3];
figure
axis([-1000, 1000, -1000, 1000, -100, 3000])
hold on;
plotobstacle(obs);
plot(kuka,q);
