%{
    Names:
    Ritvik Singh (1005834554)
    Arthur Allshire (1005713587)
    Hshmat Sahak (1005903710)
%}
%clc; clear
%close all

% create DH table (assume thetas are 0.0 for now)
DH = [0.0, 400, 25, pi/2;
    0.0, 0.0, 315, 0.0;
    0.0, 0.0, 35, pi/2;
    0.0, 365, 0.0, -pi/2;
    0.0, 0.0, 0.0, pi/2;
    0.0, 161.44, -296.23, 0.0];
% create instance of kuka
kuka = mykuka(DH);

test_H = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4]', kuka);
inverse_kuka(test_H, kuka);

H = [0, 0, 1, 500;
    0, -1, 0, 0;
    1, 0, 0, 500;
    0, 0, 0, 1];

angles = [ 0    1.5708         0         0    1.5708         0];
forward_kuka(angles', kuka)