function J = Jvi(q,myrobot,i)
    % q = [pi/4 pi/3 -pi/2 pi/4 pi/6 -pi/6];
    Hs = forwardAll(q,myrobot);
    Zs = ones(3,6);
    Os = ones(3,6);
    J = zeros(3,6);
    
    for j = 1:6
        Zs(:,j) = Hs(1:3,3,j);
        Os(:,j) = Hs(1:3,4,j);
    end
    J(:,1) = cross([0;0;1], Os(:,i));
    for j = 2:i
        J(:,j) = cross(Zs(:,j-1), Os(:,i) - Os(:,j-1));
    end
    
end

