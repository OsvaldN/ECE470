function Hs = forwardAll(joint, myrobot)
    % Initialize output with 4x4 Identity Matrix
    H = eye(4);

    for i = 1:6
        %retrieve link specific parameters
        alpha = myrobot.links(i).alpha;
        a = myrobot.links(i).a;
        d = myrobot.links(i).d;
        theta = joint(i);
        
        %Compute i-1 to i homogeneous transformation
        new_H = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); ...
            sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); ...
            0 sin(alpha) cos(alpha) d; ...
            0 0 0 1];
        
        %Multiple results with existing transform
        H = H * new_H;
        Hs[i] = H
    end
end