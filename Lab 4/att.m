function [tau] = att(q,q2,myrobot)
    % calculates the attractive component of the gradient descent algorithm.
    tau = zeros(1,6);
    
    H = cell(1, 6); % Preallocate a cell array to hold all link poses
    for i = 1:6
        H{i} = forward(q', myrobot, i);
    end
    
    H2 = cell(1, 6); % Preallocate a cell array to hold all link poses
    for i = 1:6
        H2{i} = forward(q2', myrobot, i);
    end
    
    % conic to quadratic transition 
    d = 10000;
    c = 1.0 * ones(6,1);
    
    for i = 1:6
        J_o_i = link_jac(q, i, myrobot);
        H_i = H{i};
        H_i2 = H2{i};
        o_i = H_i(1:3, 4);
        o_i2 = H_i2(1:3, 4);
        difference = o_i - o_i2;
        distance = norm(difference);
        % calculate attractive force
        if distance <= d
            F_i_att = -c(i,1)*(difference);
        else
            F_i_att = -c(i,1)*(difference)/distance * d;
        end
        % increment to total force
        tau = tau + (J_o_i' * F_i_att)';
    end
    
    % normalize
    tau = tau / norm(tau);
end

