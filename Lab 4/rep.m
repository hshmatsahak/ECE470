function [tau] = rep(q,myrobot,obs)
    % calculates the attractive component of the gradient descent algorithm.
    tau = zeros(1,6);
    
    H = cell(1, 6); % Preallocate a cell array to hold all link poses
    for i = 1:6
        H{i} = forward(q', myrobot, i);
    end
    
    rho0 = obs.rho0;
    eta = 1; % repulsive force scale factor
    
    % Loop through robot links
    for i = 1:6
        J_o_i = link_jac(q, i, myrobot);
        H_i = H{i};
        o_i = H_i(1:3, 4);
        
        % obstacle is cylinder. see prelab sheet for 3 regions.
        if strcmp(obs.type, 'cyl')
            % cylinder parameters
            h = obs.h;
            c = obs.c;
            r = obs.R;
            b = zeros(3, 1);
            c_dist = norm(o_i(1:2,1) - c);
            % If not above the cylinder
            if o_i(3, 1) <= h
                b(1:2, 1) = (r / c_dist) * (o_i(1:2,1) - c) + c;
                b(3,1) = o_i(3,1);
            % If directly above cylinder
            elseif norm(o_i(1:2, 1) - c) <= r
               b(1:2, 1) = o_i(1:2, 1);
               b(3,1) = h;
            % If above and away from cylinder
            else
                b(1:2, 1) = (r / c_dist) * (o_i(1:2,1) - c) + c;
                b(3,1) = h;
            end
        % obstacle is sphere
        elseif strcmp(obs.type, 'sph')
            c = obs.c;
            r = obs.R;
            c_diff = o_i - c;
            c_dist = norm(o_i - c);
            % connect center of sphere to o_i. Intersection with sphere is
            % b.
            b = c_diff/c_dist * r + c;
        % Obstacle is plane
        else
            plane_z = obs.h;
            b = o_i;
            % closest point is projection to plane
            b(3,1) = plane_z;
        end
        dist = norm(o_i - b);
        % if outside region of influence, ignore
        if dist > rho0
            continue
        end
        % compute repulsive force
        F_i_rep = eta*(1/dist - 1/rho0)/(dist^3) * (o_i - b);
        % increment to total force
        tau = tau + (J_o_i' * F_i_rep)';
    end
    
    % normalize
    if norm(tau) > 0
        tau = tau / norm(tau);
    end
end
