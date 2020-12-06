function Ja = ajacobian(q,myrobot)
    % q = [pi/4 pi/3 -pi/2 pi/4 pi/6 -pi/6];
    Hs = forwardAll(q,myrobot);
    
    H0_6 = Hs(:,:,6);
    R0_6 = H0_6(1:3,1:3);
    
    phi = atan2(R0_6(2,3), R0_6(1,3));
    theta = atan2( sqrt(1-(R0_6(3,3)^2)) , R0_6(3,3) );
    psi = atan2(R0_6(3,2)/sin(theta), -R0_6(3,1)/sin(theta));
    
    B = [0  -sin(phi) ( cos(phi)*sin(theta) ); ...
         0  cos(phi)  ( sin(phi)*sin(theta) ); ...
         1  0         cos(theta)          ];
    
    amat = vertcat(horzcat(eye(3),zeros(3,3)), horzcat(zeros(3,3), inv(B)));
    J = jacobian(q,myrobot);
    Ja = amat * J;
end
