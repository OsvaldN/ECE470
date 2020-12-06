function tau = att(q,q2,myrobot)
    % compute forward kinematics for both positions
    Hs_init = forwardAll(q,myrobot);
    Hs_final = forwardAll(q2,myrobot);
    zeta = 0.01;
    
    F_att = zeros(3,6);
    for i = 1:6
        
        % extract initial and desired joint positions
        Os_init(:,i) = Hs_init(1:3,4,i);
        Os_final(:,i) = Hs_final(1:3,4,i);
        
        % Compute attractive force as the negative gradient
        % of parabolic well potential
        F_att(:,i) = -zeta * (Os_init(:,i) - Os_final(:,i) );
        
    end
  
    % create placeholder for tau vector
    tau = zeros(1,6);
    
    % compute tau summation
    for i = 1:6
        % compute new jacobian and add to summation
        J = Jvi(q, myrobot, i);
        tau = tau + (J' *F_att(:,i))';
    end
    
    % normalize but don't divide by zero
    if norm(tau) ~= 0
        tau = tau / norm(tau);
    end
end

