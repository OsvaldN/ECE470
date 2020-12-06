function lab2()
    a2 = 0.01* sqrt(43.18^2 + 2.03^2);
    %DH table ordered theta, d, a, alpha by row
    DH = [0  0.76     0     pi/2; ...
          0  -0.2365  a2    0; ...
          0  0        0     pi/2; ...
          0  0.4318   0     -pi/2; ...
          0  0        0     pi/2; ...
          0  0.20     0     0];
    myrobot = mypuma560(DH);
    
    %4.1.1
    % load Href file and init qref array
    load('Href.mat')
    qref = zeros(6,100);
    for i = 1:100
        %get qref from inverse kinematics
        qref(:,i) = inverse(Href(:,:,i),myrobot);
    end
    
    
    %4.1.2a    
    odot = Href(1:3,4,:);
    % get S(omega) from Hrefdot * Href'
    Somegas = pagemtimes(Hrefdot(1:3,1:3,:), permute(Href(1:3,1:3,:), [2,1,3]));
    % extract rotational axis from S(omega) entries
    omegas = [Somegas(2,3,:); Somegas(1,3,:); Somegas(2,1,:)];
    
    %4.1.2b
    %init Jacobian inverse and qdot ref arrays
    J_qref_inv = zeros(6,6,100);
    qdref = zeros(6,100);
    for i = 1:100
        %get geometric jacobian inverse
        J_qref_inv(:,:,i) = inv(jacobian(qref(:,i),myrobot));
        %get qdref by multiplying by [odot;oemga]
        qdref(:,i) = J_qref_inv(:,:,i) * vertcat(odot(:,1,i),omegas(:,1,i));
    end
    
    %4.1.2c. euler angles
    for i = 1:100
        eulZYZ = tr2eul(Href(1:3,1:3,i));
        phi(i) = eulZYZ(1);
        theta(i) = eulZYZ(2);
        psi(i) = eulZYZ(3);
    end

    %4.1.2d
    % init alpha dot array
    alphadot = zeros(3,100);
    for i = 1:100
        % compute B(alpha) for each alpha_i
        B = [0  -sin(phi(i)) ( cos(phi(i))*sin(theta(i)) ); ...
             0  cos(phi(i))  ( sin(phi(i))*sin(theta(i)) ); ...
             1  0             cos(theta(i))               ];
        % invert B(alpha) and multiple by omega to get alphadot
        Binv = inv(B);
        alphadot(:,i) = Binv * omegas(:,1,i);
    end
    
    %4.1.2e
    %init arrays for analyic jacobian inverse and qref1
    Ja_qref_inv = zeros(6,6,100);
    qdref1 = zeros(6,100);
    for i = 1:100
        %check sin(theta) != 0
        H = forwardAll(qref(:,i),myrobot);
        if H(3,3) ~= 1
            % perform qref1 calculation if Ja is invertible
            Ja_qref_inv(:,:,i) = inv(ajacobian(qref,myrobot));
            qdref1(:,i) = Ja_qref_inv(:,:,i) * vertcat(odot(:,1,i),alphadot(:,i));
        end
    end
    
    %4.1.2f check norm between two qrefs
    norm(qdref - qdref1)
    
    %4.3 generate splines
    splineqref=spline(t,qref);
    splineqdref=spline(t,qdref1);
    
    %4.4 create cubic splines
    d=length(splineqdref.coefs);
    splineqddref=splineqdref;
    splineqddref.coefs=splineqdref.coefs(:,1:3).*(ones(d,1)*[3 2 1]);
    splineqddref.order=3;
    
    %4.2.2 simulate motors
    sys=@(t,x)motors(t,x,myrobot,splineqref,splineqdref,splineqddref);
    Ts=0.02;
    q0=[3*pi/2;zeros(11,1)];
    [t,q]=ode45(sys,[0:Ts:6*pi],q0);
    
    %4.2.3
    qref = ppval(splineqref,t)';
    
    % plot all joints, ref in blue, q in orange
    % looks like my qdref might be wrong :(
    f1 = figure('Name','Joint 1');
    plot(t,qref(:,1), t,q(:,1))
    f2 = figure('Name','Joint 2');
    plot(t,qref(:,2), t,q(:,2))
    f3 = figure('Name','Joint 3');
    plot(t,qref(:,3), t,q(:,3))
    f4 = figure('Name','Joint 4');
    plot(t,qref(:,4), t,q(:,4))
    f5 = figure('Name','Joint 5');
    plot(t,qref(:,5), t,q(:,5))
    f6 = figure('Name','Joint 6');
    plot(t,qref(:,6), t,q(:,6))
    
    %4.2.4
    %plot desired trajectory
    f7 = figure('Name','4.2.4');
    hold on;
    oref=squeeze(Href(1:3,4,:));
    plot3(oref(1,:),oref(2,:),oref(3,:),'r')
    view(-125,40);
    %plot robot
    plot(myrobot,q(:,1:6))
    hold off

end

