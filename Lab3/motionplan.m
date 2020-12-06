function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    q = q0;
    while norm(q(end,1:5) - q2(1:5)) > tol
        % compute tau
        tau = att(q(end,:), q2, myrobot);
        q(end+1,:) = q(end,:) + 0.01 * tau;
    end
    
    % impute q6 transition
    q6 = linspace(q0(6),q2(6), size(q,1));
    q(:,6) = q6;
    
    % create time variable and interpolate splines
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q');
end

