function tau = rep(q,myrobot,obs)
    % compute forward kinematics for current position
    Hs = forwardAll(q,myrobot);
    eta = 0.01;
 
    F_rep = zeros(3,6);
    for i = 1:6
    
        % extract initial joint positions
        Os(:,i) = Hs(1:3,4,i);
        
        if obs.type == 'sph'
            % get distance to centre of sphere
            dist_c = Os(:,i) - obs.c;
            %get distance to closest point
            dist_b = dist_c * (1 - obs.R/norm(dist_c));
        elseif obs.type == 'cyl'
            % same as sphere but drop z_0 dist
            dist_c = Os(1:2,i) - obs.c;
            dist_b = [dist_c * (1 - obs.R/norm(dist_c)); 0];
        end
        
        % compute gradient of point to obstacle distance
        grad_rho = dist_b / norm(dist_b);
        % compute repulsive force
        F_rep(:,i) = eta * (1/norm(dist_b) - 1/obs.rho0) * (1/(norm(dist_b)^2)) * grad_rho;
        
    end

    % create placeholder for tau vector
    tau = zeros(1,6);
    
    % compute tau summation
    for i = 1:6
        % compute new jacobian and add to summation
        J = Jvi(q, myrobot, i);
        tau = tau + (J' *F_rep(:,i))';
    end
    
    % normalize but don't divide by zero
    if norm(tau) ~= 0
        tau = tau / norm(tau);
    end
end

