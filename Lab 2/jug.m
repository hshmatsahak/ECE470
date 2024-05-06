function [] = jug(delta)


data=xlsread('jug.xlsx');
xdata=550 + 10 * data(:, 1);
ydata=10 * data(:, 2);
zdata=-ones(length(data), 1);


X_workspace= [xdata, ydata, zdata]';

X_baseframe = zeros(3, length(data));

for i = 1:length(data)
    X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
end

H = [0, 0, 1, 0;
    0, -1, 0, 0;
    1, 0, 0, 0;
    0, 0, 0, 1];

kuka = mykuka_search(delta);

for i = 1:length(data)
    
    H(1:3,4) = X_baseframe(:, i);
    
    angles = inverse_kuka(H, kuka);
    
    setAngles(angles, 0.04);
end


end