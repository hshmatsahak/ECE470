function [H] = gettf(theta, d, a, alpha)
    % calculate H^{i-1}_i from the DH parameters
    c_t = cos(theta);
    s_t = sin(theta);
    c_a = cos(alpha);
    s_a = sin(alpha);
    H = [c_t, -s_t*c_a, s_t*s_a, a*c_t;
        s_t, c_t*c_a, -c_t*s_a, a*s_t;
        0.0, s_a, c_a, d;
        0.0, 0.0, 0.0, 1.0];
end