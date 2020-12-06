function myrobot = mypuma560(DH)
    %Using SerialLink from Robotics toolbox
    myrobot = SerialLink(DH, 'name', 'Puma560');
    
    myrobot.links(1).Jm = 2e-4;
    myrobot.links(2).Jm = 2e-4;
    myrobot.links(3).Jm = 2e-4;
    myrobot.links(4).Jm = 3.3e-5;
    myrobot.links(5).Jm = 3.3e-5;
    myrobot.links(6).Jm = 3.3e-5;
    
    myrobot.links(1).B = 1.48e-3;
    myrobot.links(2).B = 8.17e-4;
    myrobot.links(3).B = 1.38e-3;
    myrobot.links(4).B = 7.12e-5;
    myrobot.links(5).B = 8.26e-5;
    myrobot.links(6).B = 3.67e-5;
    
end