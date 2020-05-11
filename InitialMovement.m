function InitialMovement(kinova,interpolation,steps)

% Robot model
robot = kinova.model;
qWait = kinova.qWait;

% Obtain current and desired joint angles
qCurrent = robot.getpos();
qDesired = qWait;

% Select the interpolation parameters
switch interpolation
    case 1
        qMatrix = jtraj(qCurrent,qDesired,steps);
    case 2
        s = lspb(0,1,steps);                  % First, create the scalar function
        qMatrix = nan(steps,6);               % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*qCurrent + s(i)*qDesired; % Generate interpolated joint angles
            end
    otherwise
        error('Interpolation = 1 for Quintic Polynomial, or 2 for Trapezoidal Velocity')
end

% Movement
for i = 1:steps
    robot.animate(qMatrix(i,:))
    drawnow();
end

end
