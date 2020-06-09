function endCell = Movement(currentPlayerLocation,playerNumber,diceNumber,board,cell,kinova,var,piece,brick,collision)
%% Player locations
for i=1
    currentCell = currentPlayerLocation(playerNumber);
    occupiedCell_1 = currentPlayerLocation(1);
    occupiedCell_2 = currentPlayerLocation(2);
    occupiedCell_3 = currentPlayerLocation(3);
    occupiedCell_4 = currentPlayerLocation(4);
end

%% Settings
for i=1
    var.robotStep = 20;
    var.plotPath = false;
end

%% Input simulation parameters
for i=1
    totalTime = 0.01; % was 0.05 (s) vid: 0.09
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
    board.side1.startCell = 1;              % Side 1 starts at cell 1
    board.side2.startCell = 11;             % Side 2 starts at cell 11
    board.side3.startCell = 21;             % Side 3 starts at cell 21
    board.side4.startCell = 31;             % Side 4 starts at cell 31
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
for j=1
    s = lspb(0,1,steps);                        % Trapezoidal trajectory scalar
    endCell = currentCell+diceNumber;           % Count the steps
    counter = 0;
    animateGuessQ = true;
    
    %Loop through the steps until reaching dice number
    while currentCell < endCell
        %Loop through parameter of each cells
        for i=1:steps
            %Check if the next cell is occupied, if it is -> offset
            if currentCell+1 == occupiedCell_1 || currentCell+1 == occupiedCell_2 || currentCell+1 == occupiedCell_3 || currentCell+1 == occupiedCell_4
                if currentCell+1 >= board.side1.startCell && currentCell+1 < board.side2.startCell %side 1
                    x(1,i) = (1-s(i))*cell{currentCell}(1,1) + s(i)*cell{currentCell+1}(1,1);               % Points in x
                    x(2,i) = (1-s(i))*cell{currentCell}(1,2) + s(i)*cell{currentCell+1}(1,2) - 0.05;        % Points in y
                    x(3,i) = 0.05 + -0.05*cos(i*delta) + var.zOffset.EEF;
                end
                if currentCell+1 >= board.side2.startCell && currentCell+1 < board.side3.startCell %side 2
                    x(1,i) = (1-s(i))*cell{currentCell}(1,1) + s(i)*cell{currentCell+1}(1,1) - 0.05;        % Points in x
                    x(2,i) = (1-s(i))*cell{currentCell}(1,2) + s(i)*cell{currentCell+1}(1,2);               % Points in y
                    x(3,i) = 0.05 + -0.05*cos(i*delta) + var.zOffset.EEF;
                end
                if currentCell+1 >= board.side3.startCell && currentCell+1 < board.side4.startCell %side 3
                    x(1,i) = (1-s(i))*cell{currentCell}(1,1) + s(i)*cell{currentCell+1}(1,1);               % Points in x
                    x(2,i) = (1-s(i))*cell{currentCell}(1,2) + s(i)*cell{currentCell+1}(1,2) + 0.05;        % Points in y
                    x(3,i) = 0.05 + -0.05*cos(i*delta) + var.zOffset.EEF;
                end
                if currentCell+1 >= board.side4.startCell && currentCell+1 < 41 %side 4
                    x(1,i) = (1-s(i))*cell{currentCell}(1,1) + s(i)*cell{currentCell+1}(1,1) + 0.05;        % Points in x
                    x(2,i) = (1-s(i))*cell{currentCell}(1,2) + s(i)*cell{currentCell+1}(1,2);               % Points in y
                    x(3,i) = 0.05 + -0.05*cos(i*delta) + var.zOffset.EEF;
                end            
            
            else
            x(1,i) = (1-s(i))*cell{currentCell}(1,1) + s(i)*cell{currentCell+1}(1,1);     % Points in x
            x(2,i) = (1-s(i))*cell{currentCell}(1,2) + s(i)*cell{currentCell+1}(1,2);     % Points in y
            x(3,i) = 0.05 + -0.05*cos(i*delta) + var.zOffset.EEF;                         % Points in z
            end
            
            %Assign EEF rotation, guessQ for each side
            if currentCell >= board.side1.startCell && currentCell < board.side2.startCell %side 1
                theta(1,i) = pi;                % Roll angle
                theta(2,i) = 0;                 % Pitch angle
                theta(3,i) = pi/2;              % Yaw angle
                guessQ = board.side1.initGuess; % Initial guess for joint angles
                phi = 0;                        % Rotation of pieces
            end
            
            if currentCell >= board.side2.startCell-2 && currentCell < board.side3.startCell %side 2 (-2 because strange movement if -1)
                theta(1,i) = pi;                % Roll angle
                theta(2,i) = 0;                 % Pitch angle
                theta(3,i) = 0;                 % Yaw angle
                guessQ = board.side2.initGuess; % Initial guess for joint angles
                phi = -pi/2;                    % Rotation of pieces
            end
            
            if currentCell >= board.side3.startCell && currentCell < board.side4.startCell %side 3
                theta(1,i) = 0;                 % Roll angle
                theta(2,i) = pi;                % Pitch angle
                theta(3,i) = pi/2;              % Yaw angle
                guessQ = board.side3.initGuess; % Initial guess for joint angles
                phi = -pi;                      % Rotation of pieces
            end
            
            if currentCell >= board.side4.startCell-2 && currentCell < 41 %side 4
                theta(1,i) = 0;                 % Roll angle
                theta(2,i) = pi;                % Pitch angle
                theta(3,i) = 0;                 % Yaw angle
                guessQ = board.side4.initGuess; % Initial guess for joint angles
                phi = -3*pi/2;                  % Rotation of pieces
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
                    if collision == true
                        % Check for collision before each step
                        result = CollisionDetection(kinova,q1,brick,true);
                        if result == true
                            EStopState(2);
                            % Update position of the brick from figure data
                            fig = findobj(gca,'Type','patch');
                            xData =  get(fig,'Xdata');
                            yData =  get(fig,'Ydata');
                            zData =  get(fig,'Zdata');
                            brick.pos = [mean(cell2mat(xData(2)),'all'), mean(cell2mat(yData(2)),'all'), mean(cell2mat(zData(2)),'all')];
                            brick.pose = makehgtform('translate',brick.pos);
                            brick.updatedPoints = (brick.pose * [brick.verts,ones(brick.vertexCount,1)]')'; 
                        end
                    end                    
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
                if collision == true
                    % Check for collision before each step
                    result = CollisionDetection(kinova,qMatrix(j,:),brick,true);
                    if result == true
                        EStopState(2);
                        % Update position of the brick from figure data
                        fig = findobj(gca,'Type','patch');
                        xData =  get(fig,'Xdata');
                        yData =  get(fig,'Ydata');
                        zData =  get(fig,'Zdata');
                        brick.pos = [mean(cell2mat(xData(2)),'all'), mean(cell2mat(yData(2)),'all'), mean(cell2mat(zData(2)),'all')];
                        brick.pose = makehgtform('translate',brick.pos);
                        brick.updatedPoints = (brick.pose * [brick.verts,ones(brick.vertexCount,1)]')'; 
                    end
                end                    
                kinova.animate(qMatrix(j,:));
                
                % Update piece location
                piece.pos = [x(1,j),x(2,j),x(3,j)-(var.zOffset.EEF-var.zOffset.pieces)];        % Define new position
                piece.pose = makehgtform('translate',piece.pos) * trotz(phi);                   % Translation * Z rotation
                piece.updatedPoints = (piece.pose * [piece.verts,ones(piece.vertexCount,1)]')'; % Transform the vertices
                piece.mesh_h.Vertices = piece.updatedPoints(:,1:3);                             % Update the mesh vertices in the patch handle
            end
            currentCell = currentCell+1; % Update current cell
            
        end
        
        %Reset counting
        for i=1
            if currentCell == 41 % Restart cell counting
                currentCell = 1;
                endCell = endCell-40;
            end
            counter = counter+1; % Display to console
        end
    end
end

%% Go home
for i=1
    currentQ = kinova.getpos();
    endQ = [currentQ(1,1), 0 0 0 0 0]; % Don't rotate base
    jointTrajectory = jtraj(currentQ,endQ,var.robotStep);
    for trajStep = 1:size(jointTrajectory,1)
        q1 = jointTrajectory(trajStep,:);
        if collision == true
            % Check for collision before each step
            result = CollisionDetection(kinova,qMatrix(j,:),brick,true);
            if result == true
                EStopState(2);
                % Update position of the brick from figure data
                fig = findobj(gca,'Type','patch');
                xData =  get(fig,'Xdata');
                yData =  get(fig,'Ydata');
                zData =  get(fig,'Zdata');
                brick.pos = [mean(cell2mat(xData(2)),'all'), mean(cell2mat(yData(2)),'all'), mean(cell2mat(zData(2)),'all')];
                brick.pose = makehgtform('translate',brick.pos);
                brick.updatedPoints = (brick.pose * [brick.verts,ones(brick.vertexCount,1)]')'; 
            end
        end                    
        kinova.animate(q1);
    end
end

end