function dx = motors(t,x,myrobot,splineqref, ...
                          splineqdref,splineqddref)

    
    % Computation of reference signal and its first two time derivatives
    
    qr=ppval(splineqref,t);
    qdr=ppval(splineqdref,t);
    qddr=ppval(splineqddref,t);

    % state vectors
    q = x(1:6);
    qd = x(7:12);
  
    for i = 1:6    
        link = myrobot.links(i);
        J(i,1) = link.Jm;
        B(i,1) = link.B;
    end

    % Controller parameters
    % Enter your gain matrices Kp and Kd here. Kp and Kd should be
    % diagonal matrices 6x6.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Kp = eye(6) .* [8e-4 8e-4 8e-4 1.32e-4 1.32e-4 1.32e-4];
    Kd = eye(6) .* [-6.8e-4 -1.7e-5 -5.8e-4 6.08e-5 4.94e-5 9.53e-5];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ft = J.*qddr + B.*qdr; % feedforward term
    Vt = ft + Kd*(qdr-qd) + Kp*(qr-q);   % feedback controller
    qdd = (Vt-B.*qd)./J;   % acceleration
    
    dx = [qd; qdd;];