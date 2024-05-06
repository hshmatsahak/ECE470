function [H] = forward(joint,myrobot)
    % calculate H^0_6
    H = eye(4);
    
    % iterate through the links to do forward kinematics
    for i = 1:6
        % get relevant DH parameter for link i
        d = myrobot.links(i).d;
        a = myrobot.links(i).a;
        alpha = myrobot.links(i).alpha;
        % calculate H^0_i
        H = H * gettf(joint(i, 1), d, a, alpha);
    end
    
end
