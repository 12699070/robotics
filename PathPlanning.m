function pathPlanning(currentCell,diceNumber)
% This function is base on Week 9 RMRC Lab solution by Jonathan   


%% Clear all & call outside functions
for i = 1
%     clear all;
    close all;
    set(0,'DefaultFigureWindowStyle','docked')
    clf
    clc
end
CellAllocation;

%% Position
teach = false; % Teach panel
zOffset = 0.035; % Offset EEF in Z-dir so it wont touch the game board
homePos = deg2rad([0 0 0 0 0 0]);

side_1_initGuess = deg2rad([-50 -78 18 90 90 0]);
side_2_initGuess = deg2rad([-140 -78 18 90 90 0]);
side_3_initGuess = deg2rad([-230 -78 18 90 90 0]);
side_4_initGuess = deg2rad([-320 -78 18 90 90 0]);

%% Simulation parameters
totalTime = 0.02; % was 0.05 (s)
controlFreq = 0.002;

%% DH parameters
for i=1
    L1 = Link('d',0.1283+0.115,     'a',0,          'alpha',pi/2,     'qlim',deg2rad([-3600 3600]),       'offset', 0);
    L2 = Link('d',0.030,            'a',0.280,      'alpha',pi,       'qlim',deg2rad([-150.1 150.1]),     'offset', pi/2);
    L3 = Link('d',0.020,            'a',0,          'alpha',pi/2,     'qlim',deg2rad([-150.1 150.1]),     'offset', pi/2);
    L4 = Link('d',0.140+0.105,      'a',0,          'alpha',pi/2,     'qlim',deg2rad([-148.98 148.98]),   'offset', pi/2);
    L5 = Link('d',0.0285+0.0285,    'a',0,          'alpha',pi/2,     'qlim',deg2rad([-144.97 145]),      'offset', pi);
    L6 = Link('d',0.105+0.130,      'a',0,          'alpha',0,        'qlim',deg2rad([-148.98 148.98]),   'offset', pi/2);
    kinova = SerialLink([L1 L2 L3 L4 L5 L6],'name','kinova');
end
%% Plot stick model
for i=1
    kinova.base(1:3,4) = [0 0 0];
    disp('Created kinova base')
        for linkIndex = 0:6
            [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['kinovaLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
            kinova.faces{linkIndex + 1} = faceData;
            kinova.points{linkIndex + 1} = vertexData;
        end
         kinova.plot3d(homePos,'workspace',[-0.4 0.4 -0.4 0.4 0 0.4]);
        axis equal;
        hold on;
        disp('Created UR3_1 robot')
        camlight
        
    if teach == true
        kinova.teach();
    end
end
%% Set parameters for the simulation
for i=1
    t = totalTime;                      % Total time (s)
    deltaT = controlFreq;               % Control frequency
    steps = t/deltaT;                   % No. of steps for simulation
    delta = 2*pi/steps;                 % Small angle change
    epsilon = 0.1;                      % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1 0.1 0.1 0.1]);      % Weighting matrix for the velocity vector
    side_1_start = 1;                   % Side 1 starts at cell 1
    side_2_start = 11;                  % Side 2 starts at cell 11
    side_3_start = 21;                  % Side 3 starts at cell 21
    side_4_start = 31;                  % Side 4 starts at cell 31
end
%% Allocate array data
for i=1
    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,6);       % Array for joint anglesR
    qdot = zeros(steps,6);          % Array for joint velocities
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    x = zeros(3,steps);             % Array for x-y-z trajectory
    positionError = zeros(3,steps); % For plotting trajectory error
    angleError = zeros(3,steps);    % For plotting trajectory error
end

%% Set up trajectory, initial pose
s = lspb(0,1,steps);                        % Trapezoidal trajectory scalar
endCell = currentCell+diceNumber;           % Count the steps
counter = 0;                                % Reset count  (to display to console)
while currentCell < endCell
    for i=1:steps
        x(1,i) = (1-s(i))*cell{currentCell}(1,1) + s(i)*cell{currentCell+1}(1,1);     % Points in x
        x(2,i) = (1-s(i))*cell{currentCell}(1,2) + s(i)*cell{currentCell+1}(1,2);     % Points in y
        x(3,i) = 0.05 + -0.05*cos(i*delta) + zOffset;                                 % Points in z
        if currentCell >= side_1_start && currentCell < side_2_start
            theta(1,i) = pi;               % Roll angle
            theta(2,i) = 0;                % Pitch angle
            theta(3,i) = pi/2;             % Yaw angle
        end
        
        if currentCell >= side_2_start-1 && currentCell < side_3_start %side_2_start-1(start rotate @cell 10)
            theta(1,i) = pi;               % Roll angle
            theta(2,i) = 0;                % Pitch angle
            theta(3,i) = 0;                % Yaw angle
        end
        
        if currentCell >= side_3_start && currentCell < side_4_start
            theta(1,i) = 0;                % Roll angle
            theta(2,i) = pi;               % Pitch angle
            theta(3,i) = pi/2;                % Yaw angle
        end
        
        if currentCell >= side_4_start-1 && currentCell < 40
            theta(1,i) = 0;               % Roll angle
            theta(2,i) = pi;                % Pitch angle
            theta(3,i) = 0;                % Yaw angle
        end
    end
    
    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1]; % Create transformation of first point and angle
    
    if currentCell >= side_1_start && currentCell < side_2_start
        guessQ = side_1_initGuess;  % Initial guess for joint angles
    end
    if currentCell >= side_2_start && currentCell < side_3_start
        guessQ = side_2_initGuess;  % Initial guess for joint angles
    end
    if currentCell >= side_3_start && currentCell < side_4_start
        guessQ = side_3_initGuess;  % Initial guess for joint angles
    end
    if currentCell >= side_4_start && currentCell < 40
        guessQ = side_4_initGuess;  % Initial guess for joint angles
    end
    qMatrix(1,:) = kinova.ikcon(T,guessQ);  % Solve joint angles to achieve first waypoint
    %% Track the trajectory with RMRC
    for i = 1:steps-1
        T = kinova.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
        deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric!
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
        xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
        J = kinova.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
        qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
        for j = 1:6                                                             % Loop through joints 1 to 6
            if qMatrix(i,j) + deltaT*qdot(i,j) < kinova.qlim(j,1)                     % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > kinova.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
        positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
        angleError(:,i) = deltaTheta;                                           % For plotting
    end
    %% Plot the results & keep count
    for i=1
        figure(1)
        plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
        kinova.animate(qMatrix);
        currentCell = currentCell+1; %update current cell
    end
    
    if currentCell == 41 % Restart cell counting
        currentCell = 1;
        endCell = endCell-40;
    end
    counter = counter+1 % Display to console
end

end






