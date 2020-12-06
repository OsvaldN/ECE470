function lab3()
    a2 = 1* sqrt(43.18^2 + 2.03^2);
    %DH table ordered theta, d, a, alpha by row
    DH = [0  76     0     pi/2; ...
          0  -23.65  a2    0; ...
          0  0        0     pi/2; ...
          0  43.18   0     -pi/2; ...
          0  0        0     pi/2; ...
          0  20     0     0];
    myrobot = mypuma560(DH);
    
    % 3.1 att.m test
    H1 = eul2tr([0 pi pi/2]);
    H1(1:3,4)=100*[-1; 3; 3;]/4;
    q1 = inverse(H1,myrobot);
    H2 = eul2tr([0 pi -pi/2]);
    H2(1:3,4)=100*[3; -1; 2;]/4;
    q2 = inverse(H2,myrobot);
    
    % check tau matches the handout
    tau = att(q1,q2,myrobot)
    
    %3.2
    qref = motionplan(q1,q2,0,10,myrobot,[],0.01);
    t=linspace(0,10,300);
    q = ppval(qref,t)';
    plot(myrobot,q)
    
    %3.3
    setupobstacle
    hold on
    axis([-1 1 -1 1 0 2])
    view(-32,50)
    plotobstacle(obs);
    qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
    t=linspace(0,10,300);
    q=ppval(qref,t)';
    plot(myrobot,q);

end

