function q_control(q1_value,q2_value,q3_value,q4_value,q5_value,q6_value)
q = [q1_value,q2_value,q3_value,q4_value,q5_value,q6_value]
L1 = Link('d',0.1283+0.115,     'a',0,          'alpha',pi/2,     'qlim',deg2rad([-154.1 154.1]),     'offset', 0);
L2 = Link('d',0.030,            'a',0.280,      'alpha',pi,       'qlim',deg2rad([-150.1 150.1]),     'offset', pi/2);
L3 = Link('d',0.020,            'a',0,          'alpha',pi/2,     'qlim',deg2rad([-150.1 150.1]),     'offset', pi/2);
L4 = Link('d',0.140+0.105,      'a',0,          'alpha',pi/2,     'qlim',deg2rad([-148.98 148.98]),   'offset', pi/2);
L5 = Link('d',0.0285+0.0285,    'a',0,          'alpha',pi/2,     'qlim',deg2rad([-144.97 145]),      'offset', pi);
L6 = Link('d',0.105+0.130,      'a',0,          'alpha',0,        'qlim',deg2rad([-148.98 148.98]),   'offset', pi/2);
kinova = SerialLink([L1 L2 L3 L4 L5 L6],'name','kinova');
kinova.animate(q);
end

function cartesian_control(xInput,yInput,zInput)

end