function [robot] = mykuka_search(delta)
links = Link.empty(0,6);
DH = [0.0, 400, 25, pi/2;
    0.0, 0.0, 315, 0.0;
    0.0, 0.0, 35, pi/2;
    0.0, 365, 0.0, -pi/2;
    0.0, 0.0, 0.0, pi/2;
    0.0, 161.44+delta(2), -296.23-delta(1), 0.0];
% create links according to DH parameters
for i = 1:6
    links(i) = Link('d', DH(i, 2), 'a', DH(i, 3), 'alpha', DH(i, 4));
end

% create robot model
robot = SerialLink(links, 'name', 'kuka');
end

