% Gradient descent to get motion plan from initial & final joint vars,
% initial and final time, obstacle and tolerance for final result
function [qref] = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    q = q0;
    alpha_i = 0.01;
    
    % Till converge
    while norm(q(end,1:5)-q2(1:5))>=tol
        q_k_prev = q(end, :);
        % attractive force given current joint vars
        total_force = att(q_k_prev, q2, myrobot);
        % Incorporate all repulsive obstacle forces
        for idx = 1:length(obs)
            rep_force = rep(q_k_prev, myrobot, obs{idx});
            total_force = total_force + rep_force;
        end
        % Grad descent step
        q_k = q_k_prev + alpha_i * total_force;
        q = [q; q_k];
    end
    
    % Cubic spline interpolation
    t = linspace(t1,t2,size(q,1));
    q(:, 6) = linspace(q0(6), q2(6), size(q,1));
    qref = spline(t,q');
end

