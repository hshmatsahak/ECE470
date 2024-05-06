function [] = mysegment(delta)

x_bar = 620;
z_bar = -1;
y_start = -100;
y_end = 100;
num_points = 100;
X_workspace= [repmat(x_bar, 1, num_points); linspace(y_start, y_end, num_points); repmat(z_bar, 1, num_points)];

X_baseframe = zeros(3, num_points);

for i = 1:num_points
    X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
end

H = [0, 0, 1, 0;
    0, -1, 0, 0;
    1, 0, 0, 0;
    0, 0, 0, 1];

kuka = mykuka_search(delta);

for i = 1:num_points
    
    H(1:3,4) = X_baseframe(:, i);
    
    angles = inverse_kuka(H, kuka);
    %H
    %angles
    setAngles(angles, 0.04);
end


end