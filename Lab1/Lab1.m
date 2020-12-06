function Lab1()
    %4.1
    % define a2 precisely with eq.
    a2 = 0.01 * sqrt(43.18^2 + 2.03^2);
    
    %DH table ordered theta, d, a, alpha by row
    DH = [0  0.76     0     pi/2; ...
          0  -0.2365  a2    0; ...
          0  0        0     pi/2; ...
          0  0.4318   0     -pi/2; ...
          0  0        0     pi/2; ...
          0  0.20     0     0];
    myrobot = mypuma560(DH);
    
    %4.2 Plot a sample joint space trajectory
    % joint trajectories
    theta1 = linspace(0, pi, 200);
    theta2 = linspace(0, pi/2, 200);
    theta3 = linspace(0, pi, 200);
    theta4 = linspace(pi/4, 3*pi/4, 200);
    theta5 = linspace(-pi/3, pi/3, 200);
    theta6 = linspace(0,2 *pi, 200);
    q = [theta1;theta2;theta3;theta4;theta5;theta6]';
    
    plot(myrobot,q)
    
    
    %4.3 Forward Kinematics
    % Results of FK model stored in o
    o = zeros(200,3);
    for i = 1:200
       temp = forward(q(i,:), myrobot);
       o(i,:) = temp(1:3,4);
    end
    
    %Plot FK output
    plot3(o(:,1),o(:,2),o(:,3),'r')
    hold on
    plot(myrobot, q)
    
    
    %4.4 Inverse Kinematics
    % define desired task space coordinates
    x = linspace(20, 30, 100);
    y = linspace(23, 30, 100);
    z = linspace(15, 100, 100);
    R = [cos(pi/4) -sin(pi/4) 0; ...
              sin(pi/4)  cos(pi/4) 0; ...
              0          0         1];
    
    % store IK model outputs
    q = zeros(1,6);
    for i = 1:100
        H = [R [x(i); y(i); z(i)]; 0 0 0 1];
        q(i,:) = inverse(H, myrobot);
    end
    
    %Compare IK model output to desired trajectory
    plot3(x, y, z,'r')
    hold on
    plot(myrobot, q)
end