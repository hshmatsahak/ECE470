function [q] = inverse(H,myrobot)
    % get DH parameters from robot
    q = zeros(1, 6);
    d = zeros(6, 1);
    a = zeros(6, 1);
    alpha = zeros(6, 1);
    for i = 1:6
        d(i, 1) = myrobot.links(i).d;
        a(i, 1) = myrobot.links(i).a;
        alpha(i, 1) = myrobot.links(i).alpha;
    end
    
    % position of wrist centre
    o_c = H(1:3, 4) - H(1:3, 1:3) * [0;0;d(6,1)];
    x_c = o_c(1);
    y_c = o_c(2);
    z_c = o_c(3);

    % inverse position kinematics
    q(1,1) = atan2(y_c, x_c) ...
            - atan2(-d(2, 1), real(sqrt(x_c^2+y_c^2 - d(2, 1)^2)));
    D = (a(2,1)^2 + d(4,1)^2 - x_c^2 - y_c^2 + d(2,1)^2 - (z_c-d(1,1))^2) ...
        / (2 * a(2, 1) * d(4, 1));
    q(1,3) = atan2(-D, real(sqrt(1-D^2)));
    q(1,2) = atan2(z_c-d(1,1), sqrt(x_c^2 + y_c^2 - d(2,1)^2)) ...
        -atan2(-d(4,1)*cos(q(1,3)), a(2,1)+d(4,1)*sin(q(1,3)));
    
    % calculate transformation from base to link 3
    H_0_3 = eye(4);
    for i = 1:3
        H_0_3 = H_0_3 * gettf(q(1, i), d(i, 1), a(i, 1), alpha(i, 1));
    end
    % extract rotation matrix
    R_0_3 = H_0_3(1:3, 1:3);
    % calculate rotation matrix from link 3 to link 6
    R_3_6 = R_0_3' * H(1:3, 1:3);
    
    % inverse orientation kinematics
    q(1, 4) = atan2(R_3_6(2,3), R_3_6(1,3));
    q(1, 5) = atan2(sqrt(1 - R_3_6(3,3)^2), R_3_6(3,3));
    q(1, 6) = atan2(R_3_6(3,2), -R_3_6(3,1));
    
end

