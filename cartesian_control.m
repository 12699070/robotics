function cartesian_control(x_value,y_value,z_value)
L1 = Link('d',0.1283+0.115,     'a',0,          'alpha',pi/2,     'qlim',deg2rad([-154.1 154.1]),     'offset', 0);
L2 = Link('d',0.030,            'a',0.280,      'alpha',pi,       'qlim',deg2rad([-150.1 150.1]),     'offset', pi/2);
L3 = Link('d',0.020,            'a',0,          'alpha',pi/2,     'qlim',deg2rad([-150.1 150.1]),     'offset', pi/2);
L4 = Link('d',0.140+0.105,      'a',0,          'alpha',pi/2,     'qlim',deg2rad([-148.98 148.98]),   'offset', pi/2);
L5 = Link('d',0.0285+0.0285,    'a',0,          'alpha',pi/2,     'qlim',deg2rad([-144.97 145]),      'offset', pi);
L6 = Link('d',0.105+0.130,      'a',0,          'alpha',0,        'qlim',deg2rad([-148.98 148.98]),   'offset', pi/2);
kinova = SerialLink([L1 L2 L3 L4 L5 L6],'name','kinova');

currentQ = kinova.getpos();
endQ = kinova.ikine(transl([x_value y_value z_value]),zeros(1,6),[1 1 1 0 0 0]);

showSteps = true; % true = show steps; false = plot straight away

if showSteps == false
    kinova.animate(endQ);
end

if showSteps == true
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end
end

end