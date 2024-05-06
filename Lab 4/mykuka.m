function [robot] = mykuka(DH)
links = Link.empty(0,6);

% create links according to DH parameters
for i = 1:6
    links(i) = Link('d', DH(i, 2), 'a', DH(i, 3), 'alpha', DH(i, 4));
end

% create robot model
robot = SerialLink(links, 'name', 'kuka');
end

