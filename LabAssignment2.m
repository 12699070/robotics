function LabAssignment2()
%% Clear
clear all; clc; clf;

% Initialize values
model = 1;      % 0 = Linear | 1 = Graphical
base = [0,0,0];

% D-H Parameters
% cd 'D:\OneDrive - UTS\41013 Robotics\Assignment 2'
switch model
    case 0
        d1 = 0.1283+0.115; d2 = 0.030; a2 = 0.280; d3 = -0.020; d4 = 0.14+0.105; d5 = 0.0285+0.0285; d6 = 0.105+0.13; % meters (m)

        L1 = Link('d',d1,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
        L2 = Link('d',d2,'a',a2,'alpha',0,'offset',pi/2,'qlim',[deg2rad(-170),deg2rad(170)]);
        L3 = Link('d',d3,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-170),deg2rad(170)]);
        L4 = Link('d',d4,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-170),deg2rad(170)]);
        L5 = Link('d',d5,'a',0,'alpha',pi/2,'offset',pi,'qlim',[deg2rad(-170),deg2rad(170)]);
        L6 = Link('d',d6,'a',0,'alpha',0,'offset',pi/2,'qlim',[deg2rad(-170),deg2rad(170)]);

        kinova.model = SerialLink([L1 L2 L3 L4 L5 L6],'name','kinova');

    case 1
        % Calling the class that creates the robot
        kinova = kinova();
        clf;
end

% Locate the robots basis
kinova.model.base = transl(base);

% Define the size of the workspace
areaSize = 0.1;
workspace = [-areaSize+base(1), areaSize+base(1), ...
            -areaSize+base(2), areaSize+base(2), ...
            0, 1];
scale = 0.5;

% Define initial joints
qo = deg2rad([0,0,0,0,0,0]); % qo = zeros(1,6);
% Define waiting joints
kinova.qWait = deg2rad([0,30,90,90,60,0]);
switch model
    case 0
        kinova.model.plot(qo,'workspace',workspace,'scale',scale);
        hold on
    case 1
        kinova.workspace = workspace;
        kinova.qo = qo;
        hold on
        kinova.PlotAndColourRobot();
        hold on
end

% Teach model
kinova.model.teach()

% Variables
% input('Press enter to begin')
stepsNormal = 25;
stepsSquares = 5;
interpolation = 2;

%% Initialization (elbow up)
display('Moving initial position (elbow up)');
InitialMovement(kinova,interpolation,stepsNormal);

% Draw hat
p1.pos = board.limits(1,:);
p1.origin = [0, 0, 0];
p1.pose = transl(p1.pos) * trotz(pi/2);
p2.pos = board.limits(1,:); + [0.05,-0.005,0];
p2.origin = [0, 0, 0];
p2.pose = transl(p2.pos) * trotz(pi/2);
p3.pos = board.limits(1,:); + [0.055,0,0];
p3.origin = [0, 0, 0];
p3.pose = transl(p3.pos) * trotz(pi/2);
p4.pos = board.limits(1,:); + [0.055,-0.005,0];
p4.origin = [0, 0, 0];
p4.pose = transl(p4.pos) * trotz(pi/2);

p1 = LocateParts(p1,'p1TopHat');
p2 = LocateParts(p2,'p2WheelBarrow');
p3 = LocateParts(p3,'p3Thimble');
p4 = LocateParts(p4,'p4Iron');

%% Movement
number = 5;
p1 = Movement(number,p1,kinova,board,interpolation,stepsNormal,stepsSquares);


end