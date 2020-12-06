function [Kp,Kd] = getGains(J,B)
    Kd = 4*J-B;
    Kp = 4*J;
end

