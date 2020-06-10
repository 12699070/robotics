function [board,cell,kinova,var,pieces,cards] = InitiateRobot()
%% Settings
for i=1
    var.plotCellMarker = false; % Plot the path
    markerSize = 50;
end

%% Input positions and offsets
for i=1
    % Offsets
    var.zOffset.EEF = 0.035;    % Offset EEF in Z-dir so it wont touch the game board
    var.zOffset.pieces = 0.005; % Offset pieces in Z-dir so they slide on the game board
    var.zOffset.Marker = 0;     %was 0
    
    % Initial joint angles
    homePos = deg2rad([0 0 0 0 0 0]);
    board.side1.initGuess = deg2rad([-50 -78 18 90 90 0]);
    board.side2.initGuess = deg2rad([-140 -78 18 90 90 0]);
    board.side3.initGuess = deg2rad([-230 -78 18 90 90 0]);
    board.side4.initGuess = deg2rad([-320 -78 18 90 90 0]);
end

%% Plot environment
for i=1
    cd Environment\     % Go to the folder Environment
    % Plot board - Structure board has all the info related with the board and its position
    board.img = imread('map.jpg');                  % Load the board image
    board.xImage = [-0.4 0.4; -0.4 0.4];            % X data for the image corners
    board.yImage = [0.4 0.4; -0.4 -0.4];            % Y data for the image corners
    board.zImage = [0.005 0.005; 0.005 0.005];      % Z data for the image corners
    surf(board.xImage,board.yImage,board.zImage,... % Plot the surface
        'CData',board.img,'FaceColor','texturemap');
    hold on;
    
    % Plot table
    table.pos = [0,0,-0.755];
    table = LocateParts(table,'table');
    
    % Plot floor
    img = imread('floor.jpg');                  % Load a sample image (floor)
    xImage = [1.5 -1.5; 1.5 -1.5];              % The x data for the image corners
    yImage = [1.5 1.5; -1.5 -1.5];              % The y data for the image corners
    zImage = [-0.75 -0.75; -0.75 -0.75];        % The z data for the image corners
    surf(xImage,yImage,zImage,...               % Plot the surface
        'CData',img,...
        'FaceColor','texturemap');
    
    % Plot background
    img = imread('background.jpg');             % Load an image (front)
    xImage = [-1.5 1.5; -1.5 1.5];              % The x data for the image corners
    yImage = [-1.5 -1.5; -1.5 -1.5];            % The y data for the image corners
    zImage = [1.5 1.5; -0.75 -0.75];            % The z data for the image corners
    surf(xImage,yImage,zImage,...               % Plot the surface
        'CData',img,...
        'FaceColor','texturemap');
    
    % Plot sensor
    sensor.pos = [0,0,-0.05];
    sensor = LocateParts(sensor,'PIR Sensor');
    
    % Plot stop button
    eStopButton.pos = [0,1.4,-0.23];
    eStopButton = LocateParts(eStopButton,'eStop');
    tablePiece.pos = [0,1.4,-0.75];
    tablePiece = LocateParts(tablePiece,'tablePiece');
        
    % Plot cards
    cards.chance.pos = [0.19,-0.19,0];
    cards.chance = LocateParts(cards.chance,'chance');
    cards.chest.pos = [-0.19,0.19,0];
    cards.chest = LocateParts(cards.chest,'chest');
    
    cd ..       % Get out of the folder
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
 
    board.side1.start = [0.270 -0.35 var.zOffset.Marker];
    board.side1.end = [-0.270 -0.35 var.zOffset.Marker];
    board.side1.yOffsetMarker = -0.35;
    board.side1.xOffsetMarker = linspace(board.side1.start(1,1),board.side1.end(1,1),numSmallCell);
    
    board.side2.start = [-0.35 -0.270 var.zOffset.Marker];
    board.side2.end = [-0.35 0.270 var.zOffset.Marker];
    board.side2.xOffsetMarker = -0.35;
    board.side2.yOffsetMarker = linspace(board.side2.start(1,2),board.side2.end(1,2),numSmallCell);
    
    board.side3.start = [-0.270 0.35 var.zOffset.Marker];
    board.side3.end = [0.270 0.35 var.zOffset.Marker];
    board.side3.yOffsetMarker = 0.35;
    board.side3.xOffsetMarker = linspace(board.side3.start(1,1),board.side3.end(1,1),numSmallCell);
    
    board.side4.start = [0.35 0.270 var.zOffset.Marker];
    board.side4.end = [0.35 -0.270 var.zOffset.Marker];
    board.side4.xOffsetMarker = 0.35;
    board.side4.yOffsetMarker = linspace(board.side4.start(1,2),board.side4.end(1,2),numSmallCell);
    
    % Assigning cells
    cellLocation = zeros(41,3); % preallocate
    % Cell 1 (Start)
    cell{1} = [0.35 -0.35 var.zOffset.Marker];
    % Cell 2 to 10 (side 1)
    for i = 2:10
        cellLocation(i,:) = [board.side1.xOffsetMarker(1,i-1),board.side1.yOffsetMarker,var.zOffset.Marker]; % pass in x,y,z
        cell{i} = cellLocation(i,:);
    end
    % Cell 11 (Jail - corner cell on bottom left)
    cell{11} = [-0.35 -0.35 var.zOffset.Marker];
    % Cell 12 to 20 (side 2)
    for i = 12:20
        cellLocation(i,:) = [board.side2.xOffsetMarker,board.side2.yOffsetMarker(1,i-11),var.zOffset.Marker]; % pass in x,y,z
        cell{i} = cellLocation(i,:);
    end
    % Cell 21 l(Red car - corner cell on top left)
    cell{21} = [-0.35 0.35 var.zOffset.Marker];
    % Cell 22 to 30 (side 3)
    for i = 22:30
        cellLocation(i,:) = [board.side3.xOffsetMarker(1,i-21),board.side3.yOffsetMarker,var.zOffset.Marker]; % pass in x,y,z
        cell{i} = cellLocation(i,:);
    end
    % Cell 31 (Arrest - corner cell on top right)
    cell{31} = [0.35 0.35 var.zOffset.Marker];
    % Cell 32 to 40 (side 4)
    for i = 32:40
        cellLocation(i,:) = [board.side4.xOffsetMarker,board.side4.yOffsetMarker(1,i-31),var.zOffset.Marker]; % pass in x,y,z
        cell{i} = cellLocation(i,:);
    end
    % Cell 41 = cell 1 (start over)
    cell{41} = cell{1};
end

%% Initiate (Plot robot and game board once)
for i=1
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
    
    % Try to color the robot - It adds the colors stored in the .ply files
    for linkIndex = 0:6
        handles = findobj('Tag', kinova.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
    
    %% Locate the pieces in the GO cell
    pieces.name = {'p1TopHat','p2WheelBarrow','p3Thimble','p4Iron'};
    % Creat offsets in the first cell
    pieces.offsets =    [0.035,0.015,var.zOffset.pieces; ...
                        0.035,-0.038,var.zOffset.pieces;...
                        -0.015,0.015,var.zOffset.pieces;...
                        -0.015,-0.038,var.zOffset.pieces];
    for j = 1:4
        fig = ['piece' num2str(j)];
        pieces.(fig).pos = cell{1} + pieces.offsets(j,:);
        pieces.(fig).origin = [0,0,0];
        pieces.(fig).name = char(pieces.name(j));
        pieces.(fig).offset = pieces.offsets(j,:);
        pieces.(fig) = LocateParts(pieces.(fig),char(pieces.name(j)));
        hold on
    end
    
    %% Plot points on map
    %4 sides
    if var.plotCellMarker == true
    plot3(board.side1.xOffsetMarker,board.side1.yOffsetMarker,var.zOffset.Marker,'.','MarkerSize',markerSize);
    plot3(board.side2.xOffsetMarker,board.side2.yOffsetMarker,var.zOffset.Marker,'.','MarkerSize',markerSize);
    plot3(board.side3.xOffsetMarker,board.side3.yOffsetMarker,var.zOffset.Marker,'.','MarkerSize',markerSize);
    plot3(board.side4.xOffsetMarker,board.side4.yOffsetMarker,var.zOffset.Marker,'.','MarkerSize',markerSize);
    %4 corners
    plot3(cell{1}(1,1),cell{1}(1,2),cell{1}(1,3),'.','MarkerSize',markerSize);
    plot3(cell{11}(1,1),cell{11}(1,2),cell{11}(1,3),'.','MarkerSize',markerSize);
    plot3(cell{21}(1,1),cell{21}(1,2),cell{21}(1,3),'.','MarkerSize',markerSize);
    plot3(cell{31}(1,1),cell{31}(1,2),cell{31}(1,3),'.','MarkerSize',markerSize);
    end
end

end