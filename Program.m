function Program(modelo,base1,base2,part1,part2,part3,partbox,stepsFast,stepsSlow)
% Clear
clc; clf;
p1.pos = part1; p2.pos = part2; p3.pos = part3; box.pos = partbox;

% model = 1; % 0 = Linear | 1 = Graphical
if modelo == '0'
    model = 0;
elseif modelo == '1'
    model = 1;
end

% Initialize values
% model = 1;      % 0 = Linear | 1 = Graphical
% base1 = [-0.23,0.23,0]; base2 = [0.23,0.23,0];
box.pos = [0, 0, 0.05];
box.origin = [0,0,0.035];
% p1.pos = [-0.42,0.46,0.04]; p2.pos = [0.22,-0.1,0]; p3.pos = [0.42,0.46,0]; % Height due to the location of the origin of p1
p1.origin = [0, 0, -0.005]; p2.origin = [0, 0, -9e-4]; p3.origin = [0, 0, 0];

union = [(base1(1)+base2(1))/2,(base1(2)+base2(2))/2+0.2,0.25];

% Definition of positions and origins of the 3d elements
ground.pos = [union(1:2),-0.5];
ground.origin = [0,0,0];

table1.pos = [base1(1:2),-0.5]; table2.pos = [base2(1:2),-0.5];
table1.origin = [0,0,0]; table2.origin = [0,0,0];

tablePiece1.pos = [p1.pos(1:2),-0.5]; tablePiece2.pos = [p2.pos(1:2),-0.5];
tablePiece3.pos = [p3.pos(1:2),-0.5]; tablePiece4.pos = [box.pos(1:2),-0.5];
tablePiece1.origin = [0,0,0]; tablePiece2.origin = [0,0,0];
tablePiece3.origin = [0,0,0]; tablePiece4.origin = [0,0,0];

p1.angleUnion = atan2((p1.pos(2)-union(2)),(p1.pos(1)-union(1)));
p2.angleUnion = atan2((p2.pos(2)-union(2)),(p2.pos(1)-union(1)));
p3.angleUnion = atan2((p3.pos(2)-union(2)),(p3.pos(1)-union(1)));

% D-H Parameters
cd 'D:\OneDrive - UTS\41013 Robotics\Assignments'
switch model
    case 0
        d1 = 0.15; a2 = -0.24; a3 = -0.21; d4 = 0.13; d5 = 0.08; d6 = 0.09; % meters (m)

        L1 = Link('d',d1,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
        L2 = Link('d',0,'a',a2,'alpha',0,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
        L3 = Link('d',0,'a',a3,'alpha',0,'offset',0,'qlim',[deg2rad(0),deg2rad(170)]);
        L4 = Link('d',d4,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
        L5 = Link('d',d5,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
        L6 = Link('d',d6,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);

        UR3_1.model = SerialLink([L1 L2 L3 L4 L5 L6],'name','UR3_1');
        UR3_2.model = SerialLink([L1 L2 L3 L4 L5 L6],'name','UR3_2');

    case 1
        % Calling the class that creates the robot
        UR3_1 = UR3(1); 
        UR3_2 = UR3(2);
        clf;
end

% Define two different colors
UR3_1.model.comment = 'c.';
UR3_2.model.comment = 'g.';
% Locate the robots basis
UR3_1.model.base = transl(base1);
UR3_2.model.base = transl(base2);

% Define the size of the workspace
areaSize = 0.68;
workspace = [-areaSize+(min(base1(1),base2(1))), areaSize+(max(base1(1),base2(1))), ...
            -areaSize+(min(base1(2),base2(2))), areaSize+(max(base1(2),base2(2))), ...
            -0.5, 0.8];
scale = 0.5;

% Locating the fences and defining its origins
sep = 0.1;
fence11.pos = [(workspace(1)+workspace(2))/2,workspace(3)+sep,0];
fence12.pos = [(workspace(1)+workspace(2))/2,workspace(4)-sep,0];
fence21.pos = [workspace(1)+sep,(workspace(3)+workspace(4))/2,0];
fence22.pos = [workspace(2)-sep,(workspace(3)+workspace(4))/2,0];
fence11.origin = [0,0,-workspace(5)]; fence12.origin = [0,0,-workspace(5)]; 
fence21.origin = [0,0,-workspace(5)]; fence22.origin = [0,0,-workspace(5)]; 

% Selection of robots
[p1,p2,p3] = RobotSelection(p1,p2,p3,UR3_1,UR3_2);

% Obtain angles between pieces and robots
p1.angle = atan2((p1.pos(2)-p1.robot.model.base(2,4)),(p1.pos(1)-p1.robot.model.base(1,4)));
p3.angle = atan2((p3.pos(2)-p3.robot.model.base(2,4)),(p3.pos(1)-p3.robot.model.base(1,4)));

% Define transforms for the end-effector to grab
p2.pose = transl(p2.pos) * trotx(pi);
p1.pose = transl(p1.pos) * trotx(-pi/2) * troty(-(p1.angle - pi/2));
p3.pose = transl(p3.pos) * trotx(-pi/2) * troty(-(p3.angle - pi/2));

% trplot(p1.pose)
% hold on
% trplot(p3.pose)

% Plot both robots
% Define initial joints
qo = deg2rad([0,-90,0,0,-90,0]); % qo = zeros(1,6);
switch model
    case 0
        UR3_1.model.plot(qo,'workspace',workspace,'scale',scale);
        hold on
        UR3_2.model.plot(qo,'workspace',workspace,'scale',scale);
        % UR3_1.model.teach()
        % UR3_2.model.teach()
    case 1
        UR3_1.workspace = workspace;
        UR3_2.workspace = workspace;
        UR3_1.qo = qo;
        UR3_2.qo = qo;
        UR3_1.PlotAndColourRobot();
        hold on
        UR3_2.PlotAndColourRobot();
end

% Locate pieces
hold on
p1 = LocateParts(p1,'1');
p2 = LocateParts(p2,'2');
p3 = LocateParts(p3,'3');
ground = LocateParts(ground,'ground');
table1 = LocateParts(table1,'table');
table2 = LocateParts(table2,'table');
tablePiece1 = LocateParts(tablePiece1,'TablePiece');
tablePiece2 = LocateParts(tablePiece2,'TablePiece');
tablePiece3 = LocateParts(tablePiece3,'TablePiece');
tablePiece4 = LocateParts(tablePiece4,'TablePiece');
fence11 = LocateParts(fence11,'fence1');
% fence12 = LocateParts(fence12,'fence1');
fence21 = LocateParts(fence21,'fence2');
fence22 = LocateParts(fence22,'fence2');
box = LocateParts(box,'box');

%% Movements
input('Press enter to begin')
% stepsFast = 25;
% stepsSlow = 50;
interpolation = 2;
%% Initialization (elbow up)
display('Moving initial position (elbow up)');
qInit1 = deg2rad([0,-70,110,-128,90,0]);
qInit2 = deg2rad([0,-70,110,-128,-90,0]);

InitialMovement(qInit1,interpolation,stepsFast,p1.robot.model);
InitialMovement(qInit2,interpolation,stepsFast,p2.robot.model);

%% Picking up Housing Top and PCB
display('Picking up Housing Top');
p1.piece = 0; p2.piece = 0;
p1 = Movement(p1,interpolation,stepsFast);
Represent(p1)
% trplot(p1.robot.model.fkine(p1.robot.model.getpos()))
% trplot(p1.pose)
% trplot(p1.pieceTr)
display('Picking up PCB');
p2 = Movement(p2,interpolation,stepsFast);
Represent(p2)

%% Moving pieces to assembling location
p1.piece = 1; p2.piece = 1;
% p1.robot.model.base()
display('Moving Housing Top to assembling location');
p1.union = union - [0, 0, 0]; % max(p1.verts,[],1)-min(p1.verts,[],1)];
p1.unionTr = transl(p1.union) * trotz((1-(double(pi/2<abs(p1.angleUnion))*2))*pi/2);% * trotx(-pi/2)
p1 = Movement(p1,interpolation,stepsFast);
Represent(p1)

display('Moving PCB to assembling location');
p2.union = union + [0, 0, 0.15]; 
p2.unionTr = transl(p2.union) * trotz((-1+(double(pi/2<abs(p2.angleUnion))*2))*pi/2); %* trotx(-pi/2)
p2 = Movement(p2,interpolation,stepsFast);
Represent(p2)

%% Approximation
display('Dropping PCB (slow approach)');
p2.piece = 2;
p2.union = union;
p2.unionTr = transl(p2.union) * trotz((-1+(double(pi/2<abs(p2.angleUnion))*2))*pi/2);
p2 = Movement(p2,interpolation,stepsSlow);

%% Picking up Housing Bottom
display('Picking up Housing Bottom')
p3.piece = 0;
p3 = Movement(p3,interpolation,stepsFast);
Represent(p3)
% InitialMovement(qInit2,interpolation,steps,p3.robot.model);

%% Moving Housing Bottom to assembling location
p3.piece = 1;
display('Moving Housing Bottom to assembling location');
p3.union = union + [0, 0, 0.15];
p3.unionTr = transl(p3.union) * trotz((1-(double(pi/2<abs(p3.angleUnion))*2))*pi/2);
p3 = Movement(p3,interpolation,stepsFast);
Represent(p3)

%% Approximation
display('Dropping Housing Bottom (slow approach)');
p3.piece = 2;
p3.union = union;
p3.unionTr = transl(p3.union) * trotz((1-(double(pi/2<abs(p3.angleUnion))*2))*pi/2);
p3 = Movement(p3,interpolation,stepsSlow);

%% Final Movement
display('UR3_2 moves to initial pose');
InitialMovement(qo,interpolation,stepsFast,p3.robot.model);
Represent(p3)

display('Dropping off assembled piece in the box');
p1.boxTr = transl(box.pos);
FinalMovement(p1,p2,p3,interpolation,stepsFast);
Represent(p1)

display('UR3_1 moves to initial pose');
InitialMovement(qo,interpolation,stepsFast,p1.robot.model);
Represent(p1)

%% Area of Movements
input('Press enter to obtain the volume of movements')
% Define the number of steps each joint will be evaluated
stepsFast = deg2rad(30);
radius = 0.15;
% Call the function
VolumeOfMovement(UR3_1.model,UR3_2.model,stepsFast,radius)
VolumeOfMovement(UR3_2.model,UR3_1.model,stepsFast,radius)




end