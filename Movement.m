function endCell = Movement(currentCell,diceNumber,board,cell,kinova,var,piece,brick)

%% Input simulation parameters
for i=1
    totalTime = 0.02; % was 0.05 (s)
    controlFreq = 0.002;
end

%% Set parameters for the simulation
for i=1
    t = totalTime;                      % Total time (s)
    deltaT = controlFreq;               % Control frequency
    steps = t/deltaT;                   % No. of steps for simulation
    delta = 2*pi/steps;                 % Small angle change
    epsilon = 0.1;                      % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1 0.1 0.1 0.1]);      % Weighting matrix for the velocity vector
    board.side1.start = 1;              % Side 1 starts at cell 1
    board.side2.start = 11;             % Side 2 starts at cell 11
    board.side3.start = 21;             % Side 3 starts at cell 21
    board.side4.start = 31;             % Side 4 starts at cell 31
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
            x(1,i) = (1-s(i))*cell{currentCell}(1,1) + s(i)*cell{currentCell+1}(1,1) + piece.offset(1);     % Points in x
            x(2,i) = (1-s(i))*cell{currentCell}(1,2) + s(i)*cell{currentCell+1}(1,2) + piece.offset(2);     % Points in y
            x(3,i) = 0.05 + -0.05*cos(i*delta) + var.zOffset.EEF;                                 % Points in z
            if currentCell >= board.side1.start && currentCell < board.side2.start
                theta(1,i) = pi;               % Roll angle
                theta(2,i) = 0;                % Pitch angle
                theta(3,i) = pi/2;             % Yaw angle
                rotat = 0;
            end
            
            if currentCell >= board.side2.start-1 && currentCell < board.side3.start %board.side2.start-1(start rotate @cell 10)
                theta(1,i) = pi;               % Roll angle
                theta(2,i) = 0;                % Pitch angle
                theta(3,i) = 0;                % Yaw angle
                rotat = -pi/2;
            end
            
            if currentCell >= board.side3.start && currentCell < board.side4.start
                theta(1,i) = 0;                % Roll angle
                theta(2,i) = pi;               % Pitch angle
                theta(3,i) = pi/2;                % Yaw angle
                rotat = -pi;
            end
            
            if currentCell >= board.side4.start-1 && currentCell < 40
                theta(1,i) = 0;               % Roll angle
                theta(2,i) = pi;                % Pitch angle
                theta(3,i) = 0;                % Yaw angle
                rotat = -2*pi/3;
            end
        end
        
        %Assign initial guess Q
        for i=1
            if board.side1.start <= currentCell  && currentCell < board.side2.start
                guessQ = board.side1.initGuess;  % Initial guess for joint angles
            end
            if board.side2.start <= currentCell && currentCell < board.side3.start
                guessQ = board.side2.initGuess;  % Initial guess for joint angles
            end
            if board.side3.start <= currentCell && currentCell < board.side4.start
                guessQ = board.side3.initGuess;  % Initial guess for joint angles
            end
            if board.side4.start <= currentCell && currentCell < 40
                guessQ = board.side4.initGuess;  % Initial guess for joint angles
            end
        end
        
        % Go to pickup
        for i=1
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1]; % Create transformation of first point and angle
            qMatrix(1,:) = kinova.ikcon(T,guessQ);  % Solve joint angles to achieve first waypoint
            if animateGuessQ == true
                currentQ = kinova.getpos();
                endQ = qMatrix(1,:);
                jointTrajectory = jtraj(currentQ,endQ,var.robotStep);
                for trajStep = 1:size(jointTrajectory,1)
                    q1 = jointTrajectory(trajStep,:);
                    IsCollision(kinova,q1,brick,false);
                    kinova.animate(q1);
                end
                animateGuessQ = false; %once
            end
        end
        
        % Track the trajectory with RMRC
        for k=1
            for i = 1:steps-1
                T = kinova.fkine(qMatrix(i,:));                         % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                           % Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                            % Calculate rotation matrix error
                S = Rdot*Ra';                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];            % Calculate end-effector velocity to reach next waypoint.
                J = kinova.jacob0(qMatrix(i,:));                        % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon                           % If manipulability is less than given threshold
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
            if var.plotPath == true
                plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
            end
            for j = 1:size(qMatrix)
                result = IsCollision(kinova,qMatrix(j,:),brick,false);
                kinova.animate(qMatrix(j,:));
                piece.pos = [x(1,j),x(2,j),x(3,j)];
                piece.pose = makehgtform('translate',piece.pos);
%                 try piece.pose = piece.pose * trotz(); end
                % Transform the vertices
                piece.updatedPoints = (piece.pose * trotz(rotat) * [piece.verts,ones(piece.vertexCount,1)]')';
                % Update the mesh vertices in the patch handle
                piece.mesh_h.Vertices = piece.updatedPoints(:,1:3);
            end
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
    jointTrajectory = jtraj(currentQ,endQ,var.robotStep);
    for trajStep = 1:size(jointTrajectory,1)
        q1 = jointTrajectory(trajStep,:);
        result = IsCollision(kinova,qMatrix(j,:),brick,false);
        kinova.animate(q1);
    end
end

end