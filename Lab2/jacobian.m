function J = jacobian(q,myrobot)
    % q = [pi/4 pi/3 -pi/2 pi/4 pi/6 -pi/6];
    Hs = forwardAll(q,myrobot);
    Zs = ones(3,6);
    Os = ones(3,6);
    Jv = ones(3,6);
    
    for i = 1:6
        Zs(:,i) = Hs(1:3,3,i);
        Os(:,i) = Hs(1:3,4,i);
    end
    Jv(:,1) = cross([0;0;1], Os(:,6));
    for i = 2:6
        Jv(:,i) = cross(Zs(:,i-1), Os(:,6) - Os(:,i-1));
    end
    
    J = vertcat(Jv,[[0;0;1] Zs(:,1:5)]);
end

