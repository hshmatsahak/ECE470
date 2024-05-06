function [J] = link_jac(q,idx,myrobot)
    % Calculates the Jacobian J_{o_{idx}}
    % Assumes every joint is revolute

    J = zeros(3,6);
    
    H = cell(1, idx); % Preallocate a cell array to hold all link poses
    for i = 1:idx
        H{i} = forward(q', myrobot, i);
    end
    
    H_i = H{idx};
    o_i = H_i(1:3, 4);
    
    for j = 1:idx
        if j == 1
            z_0 = [0; 0; 1];
            J(:, j) = cross(z_0, o_i);
        else
            H_j_prev = H{j-1};
            z_j_prev = H_j_prev(1:3, 3);
            o_j_prev = H_j_prev(1:3, 4);
            J(:, j) = cross(z_j_prev, o_i - o_j_prev);
        end
    end
end

