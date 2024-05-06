function [H] = forward(joint,myrobot, final_link)
    % calculate H^0_{final_link}
    H = eye(4);
    % iterate through the links to do forward kinematics
    for i = 1:final_link
        % get relevant DH parameter for link i
        d = myrobot.links(i).d;
        a = myrobot.links(i).a;
        alpha = myrobot.links(i).alpha;
        % calculate H^0_i
        H = H * gettf(joint(i, 1), d, a, alpha);
    end
    
end
