function [] = mycircle(delta)

center_x = 620;
center_y = 0;
center_z = -1;
radius = 50;

num_points = 100;


theta = linspace(0, 2 * pi, num_points);

X_workspace= [repmat(center_x, 1, num_points) + radius * cos(theta); repmat(center_y, 1, num_points) + radius * sin(theta); repmat(center_z, 1, num_points)];

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
    
    angles
    H
    %setAngles(angles, 0.04);
end


end