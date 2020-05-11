function piece = Movement(number,piece,kinova,board,interpolation,stepsNormal,stepsSquare)

% Select the robot
robot = kinova.model;

% Define current joint angles
qCurrent = robot.getpos();

% Obtain current pose of the object
desiredTr = piece.pose * trotx(pi); %* trotz(-pi/2);
qDesired = robot.ikcon(desiredTr,[qCurrent(1),0,qCurrent(3),qCurrent(4),qCurrent(5),0]);

% Obtain sizes board
limits = board.limits;
length = board.length;
square = board.square;

% Select the interpolation parameters
switch interpolation
    case 1
        qMatrix = jtraj(qCurrent,qDesired,stepsNormal);
%         qMatrix(steps/3:2*steps/3,:) = jtraj(qCurrent,qDesired,steps/3);  % Two trajectories to increase precision
    case 2
        s = lspb(0,1,stepsNormal);                  % First, create the scalar function
        qMatrix = nan(stepsNormal,6);               % Create memory allocation for variables
            for i = 1:stepsNormal
                qMatrix(i,:) = (1-s(i))*qCurrent + s(i)*qDesired; % Generate interpolated joint angles
            end
    otherwise
        error('Interpolation = 1 for Quintic Polynomial, or 2 for Trapezoidal Velocity')
end

% Initial movement
for i = 1:stepsNormal
    robot.animate(qMatrix(i,:))
    drawnow();
end

% Transform between the current end-eff tf and the tf of the piece
piece.relation = inv(robot.fkine(robot.getpos())) * piece.pose; % Relation bw initial poseTr and not just pose (determines pose of end-eff)

for i = 1:number
    proof = piece.pose * transl(square,0,0);
    if proof(1,4) < limits(1,1) || limits(2,1) < proof(1,4) || ...
            proof(2,4) < limits(1,2) || limits(2,2) < proof(2,4) 
        piece.pose = piece.pose * trotz(-pi/2);
    end
    for j = 1:stepsSquare
        % New transform of the piece
    %     desiredTr = robot.fkine(robot.getpos()) * transl(square/steps,0,0);

        piece.pose = piece.pose * transl(square/stepsSquare,0,0);
        % New transform of the eff
        desiredTr = piece.pose * inv(piece.relation);
        % Joint angles
        qDesired = robot.ikcon(desiredTr,qDesired);
        % Animate robot
        robot.animate(qDesired)

        % Transform the vertices
        piece.updatedPoints = (piece.pose * [piece.verts,ones(piece.vertexCount,1)]')';
        % Update the mesh vertices in the patch handle
        piece.mesh_h.Vertices = piece.updatedPoints(:,1:3);

        drawnow();
    end
end

qCurrent = robot.getpos();
qDesired = kinova.qWait;

% Select the interpolation parameters
switch interpolation
    case 1
        qMatrix = jtraj(qCurrent,qDesired,stepsNormal);
    case 2
        s = lspb(0,1,stepsNormal);                  % First, create the scalar function
        qMatrix = nan(stepsNormal,6);               % Create memory allocation for variables
            for i = 1:stepsNormal
                qMatrix(i,:) = (1-s(i))*qCurrent + s(i)*qDesired; % Generate interpolated joint angles
            end
    otherwise
        error('Interpolation = 1 for Quintic Polynomial, or 2 for Trapezoidal Velocity')
end

% Movement
for i = 1:stepsNormal
    robot.animate(qMatrix(i,:))
    drawnow();
end
end
