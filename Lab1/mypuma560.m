function myrobot = mypuma560(DH)
    %Using SerialLink from Robotics toolbox
    myrobot = SerialLink(DH, 'name', 'Puma560');
end