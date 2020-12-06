function q = inverse(H, myrobot)
    % retrieve relevant parameters
    a2 = myrobot.links(2).a;
    d1 = myrobot.links(1).d;
    d2 = myrobot.links(2).d;
    d4 = myrobot.links(4).d;
    d6 = myrobot.links(6).d;
    R6_0 = H(1:3,1:3);
    o6_0 = H(1:3,4);
    
    % compute wrist position
    oc_0 = o6_0 - (R6_0 * [0 ;0; d6]);
    xc = oc_0(1);
    yc = oc_0(2);
    zc = oc_0(3);
    
    % Compute Theta1 with projection to x_0-y_0 plane
    % all thetas are stored immediately to q array
    l = sqrt(xc^2 + yc^2);
    % Beta is the portion of offset determined by d2 and (xc,yc)
    beta = asin(-d2 / l);
    q(1) = atan2(yc, xc) - beta;
    
    % s is the height relative to frame 1
    s = zc - d1;
    % r is the projection of the (x,yc) vector onto the x1-y1 plane
    r = l * cos(beta);
    % D is equal to sin(theta3)
    D = (r^2 + s^2 - (a2^2) - (d4^2)) / (2 * a2 * d4);
    q(3) = atan2(D, real(sqrt(1-D^2)));
    
    % psi is the full angle from x1 to O_c
    psi = atan2(s, r);
    % gamma is the portion of psi that is not theta2
    gamma = atan2(d4*sin(q(3) - pi/2), a2 + d4*cos(q(3) - pi/2));
    q(2) = psi - gamma;
    
    % compute R_3_wrt_0
    H3_0 = eye(4);
    % use FK method to get H3_0
    for i = 1:3
        alpha = myrobot.links(i).alpha;
        a = myrobot.links(i).a;
        d = myrobot.links(i).d;
        theta = q(i);
        
        new_H = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); ...
            sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); ...
            0 sin(alpha) cos(alpha) d; ...
            0 0 0 1];
        H3_0 = H3_0 * new_H;
    end
    
    % Pull out R3_0 from H3_0
    R3_0 = H3_0(1:3,1:3);
    % get R6_3 using R0_3 and R6_0
    R6_3 = R3_0' * R6_0;
    
    eulZYZ = rotm2eul(R6_3,'ZYZ');
    q(4) = eulZYZ(1);
    q(5) = eulZYZ(2);
    q(6) = eulZYZ(3);
    % get wrist orientation using Euler Angles method from class
    %q(4) = atan2(R6_3(2,3), R6_3(1,3));
    %q(5) = atan2(sqrt(1 - R6_3(3,3)^2), R6_3(3,3));
    %q(6) = atan2(R6_3(3,2), R6_3(3,1));
    

end