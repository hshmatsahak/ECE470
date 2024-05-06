function [qref] = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    % Initialize variables
    q = q0;
    alpha_rep = 0.01; % Scale factor for repulsive force
    alpha_att = 0.01; % Scale factor for attractive force (0.013 for prelab).
    total_weight = 1; % Total force scale factor
    num_iters = 0;
    
    % Loop while not converged
    while norm(q(end,1:5)-q2(1:5))>=tol && num_iters < 10000
        fprintf("\nError: %f\n", norm(q(end,1:5)-q2(1:5)));
        q_k_prev = q(end, :);
        % Find attraction force
        total_force = alpha_att*att(q_k_prev, q2, myrobot);
        % Sum repulsive forces from each obstacle
        for idx = 1:length(obs)
            rep_force = rep(q_k_prev, myrobot, obs{idx});
            total_force = total_force + alpha_rep * rep_force;
        end
        % Update with new joint angles
        q_k = q_k_prev + total_weight * total_force;
        q = [q; q_k];
        num_iters = num_iters + 1;
    end

    t = linspace(t1,t2,size(q,1));
    
    % linspace to move last joint axis from start position to end position
    q(:, 6) = linspace(q0(6), q2(6), size(q,1))
    
    % Cubic spline interpolation of joint angles to times t
    qref = spline(t,q');
end

