function [tau] = rep(q,myrobot,obs)
    % calculates the repulsive component of the gradient descent algorithm.
    tau = zeros(1,6);
    
    H = cell(1, 6); % Preallocate a cell array to hold all link poses
    for i = 1:6
        H{i} = forward(q', myrobot, i);
    end
    
    % Obstacles params
    rho0 = obs.rho0;
    c = obs.c;
    r = obs.R;
    
    % Loop through joints
    for i = 1:6
        J_o_i = link_jac(q, i, myrobot);
        H_i = H{i};
        o_i = H_i(1:3, 4);
        % Cylinder
        if strcmp(obs.type, 'cyl')
            dist = norm(o_i(1:2,1) - c) - r;
            if dist > rho0
                continue
            end
            % b is closest point on cylinder to o_i
            b = zeros(3, 1);
            b(3,1) = o_i(3,1);
            b(1:2, 1) = r / dist * (o_i(1:2,1) - c) + c;
        % Sphere
        else
            dist = norm(o_i - c) - r;
            if dist > rho0
                continue
            end
            c_diff = o_i - c;
            c_dist = norm(o_i - c);
            % b is closest point on sphere to o_i
            b = c_diff/c_dist * r + c;
        end
        % Plug into formula
        F_i_rep = (1/dist - 1/rho0)/(dist^3) * (o_i - b);
        % increment to total force
        tau = tau + (J_o_i' * F_i_rep)';
    end
    
    % normalize
    tau = tau / (norm(tau)+0.00000001);
end

