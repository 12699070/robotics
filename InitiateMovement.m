function Initiate(currentCell,diceNumber,doneInit)

%% Settings
for i=1
    robotStep = 20;
    markerSize = 50;
    plotPath = true;
end

%% Input positions and offsets
for i=1
    zOffsetEEF = 0.035; % Offset EEF in Z-dir so it wont touch the game board
    zOffsetMarker = 0; %was 0
    
    homePos = deg2rad([0 0 0 0 0 0]);
    side_1_initGuess = deg2rad([-50 -78 18 90 90 0]);
    side_2_initGuess = deg2rad([-140 -78 18 90 90 0]);
    side_3_initGuess = deg2rad([-230 -78 18 90 90 0]);
    side_4_initGuess = deg2rad([-320 -78 18 90 90 0]);
end

%% Input simulation parameters
for i=1
    totalTime = 0.02; % was 0.05 (s)
    controlFreq = 0.002;
end

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

%% Mapping
for i=1
    %Create equal points on 4 sides
    numSmallCell = 9;
    side_1_start = [0.270 -0.35 zOffsetMarker];
    side_1_end = [-0.270 -0.35 zOffsetMarker];
    side_1_yOffsetMarker = -0.35;
    side_1_xOffsetMarker = linspace(side_1_start(1,1),side_1_end(1,1),numSmallCell);
    
    side_2_start = [-0.35 -0.270 zOffsetMarker];
    side_2_end = [-0.35 0.270 zOffsetMarker];
    side_2_xOffsetMarker = -0.35;
    side_2_yOffsetMarker = linspace(side_2_start(1,2),side_2_end(1,2),numSmallCell);
    
    side_3_start = [-0.270 0.35 zOffsetMarker];
    side_3_end = [0.270 0.35 zOffsetMarker];
    side_3_yOffsetMarker = 0.35;
    side_3_xOffsetMarker = linspace(side_3_start(1,1),side_3_end(1,1),numSmallCell);
    
    side_4_start = [0.35 0.270 zOffsetMarker];
    side_4_end = [0.35 -0.270 zOffsetMarker];
    side_4_xOffsetMarker = 0.35;
    side_4_yOffsetMarker = linspace(side_4_start(1,2),side_4_end(1,2),numSmallCell);
    
    %Assigning cells
    cellLocation = zeros(41,3); %preallocate
    % Cell 1 (Start)
    cell{1} = [0.35 -0.35 zOffsetMarker];
    % Cell 2 to 10 (side 1)
    for i = 2:10
        cellLocation(i,:) = [side_1_xOffsetMarker(1,i-1),side_1_yOffsetMarker,zOffsetMarker]; % pass in x,y,z
        cell{i} = cellLocation(i,:);
    end
    % Cell 11 (Jail - corner cell on bottom left)
    cell{11} = [-0.35 -0.35 zOffsetMarker];
    % Cell 12 to 20 (side 2)
    for i = 12:20
        cellLocation(i,:) = [side_2_xOffsetMarker,side_2_yOffsetMarker(1,i-11),zOffsetMarker]; % pass in x,y,z
        cell{i} = cellLocation(i,:);
    end
    % Cell 21 l(Red car - corner cell on top left)
    cell{21} = [-0.35 0.35 zOffsetMarker];
    % Cell 22 to 30 (side 3)
    for i = 22:30
        cellLocation(i,:) = [side_3_xOffsetMarker(1,i-21),side_3_yOffsetMarker,zOffsetMarker]; % pass in x,y,z
        cell{i} = cellLocation(i,:);
    end
    % Cell 31 (Arrest - corner cell on top right)
    cell{31} = [0.35 0.35 zOffsetMarker];
    % Cell 32 to 40 (side 4)
    for i = 32:40
        cellLocation(i,:) = [side_4_xOffsetMarker,side_4_yOffsetMarker(1,i-31),zOffsetMarker]; % pass in x,y,z
        cell{i} = cellLocation(i,:);
    end
    % Cell 41 = cell 1 (start over)
    cell{41} = cell{1};
end

%% Initiate (Plot robot and game board once)
for i=1
    if doneInit == false
        %Plot Kinova
        kinova.base(1:3,4) = [0 0 0];
        disp('Created kinova base')
        for linkIndex = 0:6
            [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['kinovaLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
            kinova.faces{linkIndex + 1} = faceData;
            kinova.points{linkIndex + 1} = vertexData;
        end
        kinova.plot3d(homePos,'workspace',[-0.4 0.4 -0.4 0.4 0 0.4]);
        disp('Created Kinova robot')
        axis equal;
        camlight
        hold on;
        
        %Plot game board
        img = imread('map.jpg');          % Load a sample image (floor)
        xImage = [-0.4 0.4; -0.4 0.4];      % The x data for the image corners
        yImage = [0.4 0.4; -0.4 -0.4];          % The y data for the image corners
        zImage = [0.005 0.005; 0.005 0.005];% The z data for the image corners
        surf(xImage,yImage,zImage,...       % Plot the surface
            'CData',img,...
            'FaceColor','texturemap');
        hold on;
        
        %Plot points on map
        %4 sides
        plot3(side_1_xOffsetMarker,side_1_yOffsetMarker,zOffsetMarker,'.','MarkerSize',markerSize);
        plot3(side_2_xOffsetMarker,side_2_yOffsetMarker,zOffsetMarker,'.','MarkerSize',markerSize);
        plot3(side_3_xOffsetMarker,side_3_yOffsetMarker,zOffsetMarker,'.','MarkerSize',markerSize);
        plot3(side_4_xOffsetMarker,side_4_yOffsetMarker,zOffsetMarker,'.','MarkerSize',markerSize);
        %4 corners
        plot3(cell{1}(1,1),cell{1}(1,2),cell{1}(1,3),'.','MarkerSize',markerSize);
        plot3(cell{11}(1,1),cell{11}(1,2),cell{11}(1,3),'.','MarkerSize',markerSize);
        plot3(cell{21}(1,1),cell{21}(1,2),cell{21}(1,3),'.','MarkerSize',markerSize);
        plot3(cell{31}(1,1),cell{31}(1,2),cell{31}(1,3),'.','MarkerSize',markerSize);
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

%% Animate RMRC
for i=1
    s = lspb(0,1,steps);                        % Trapezoidal trajectory scalar
    endCell = currentCell+diceNumber;           % Count the steps
    counter = 0;
    animateGuessQ = true;
    
    %Loop through the steps until reaching dice number
    while currentCell < endCell
        %Loop through parameter of each cells
        for i=1:steps
            x(1,i) = (1-s(i))*cell{currentCell}(1,1) + s(i)*cell{currentCell+1}(1,1);     % Points in x
            x(2,i) = (1-s(i))*cell{currentCell}(1,2) + s(i)*cell{currentCell+1}(1,2);     % Points in y
            x(3,i) = 0.05 + -0.05*cos(i*delta) + zOffsetEEF;                                 % Points in z
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
        
        %Assign initial guess Q
        for i=1
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
        end
        
        % Go to pickup
        for i=1
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1]; % Create transformation of first point and angle
            qMatrix(1,:) = kinova.ikcon(T,guessQ);  % Solve joint angles to achieve first waypoint
            if animateGuessQ == true
                currentQ = kinova.getpos();
                endQ = qMatrix(1,:);
                jointTrajectory = jtraj(currentQ,endQ,robotStep);
                for trajStep = 1:size(jointTrajectory,1)
                    q1 = jointTrajectory(trajStep,:);
                    kinova.animate(q1);
                end
                animateGuessQ = false; %once
            end
        end
        
        % Track the trajectory with RMRC
        for i=1
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
        end
        
        %Animate, plot path, keep count
        for i=1
            figure(1)
            if plotPath == true
                plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
            end
            kinova.animate(qMatrix);
            currentCell = currentCell+1; %update current cell
        end
        
        %Reset counting
        for i=1
            if currentCell == 41 % Restart cell counting
                currentCell = 1;
                endCell = endCell-40;
            end
            counter = counter+1 % Display to console
        end
    end
end

%% Go home
for i=1
    currentQ = kinova.getpos();
    endQ = [currentQ(1,1), 0 0 0 0 0]; %Don't rotate base
    jointTrajectory = jtraj(currentQ,endQ,robotStep);
    for trajStep = 1:size(jointTrajectory,1)
        q1 = jointTrajectory(trajStep,:);
        kinova.animate(q1);
    end
end

end