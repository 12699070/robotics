%% Settings
zOffset = 0; %was 0
markerSize = 50;

%% Plot map
for i=1
img = imread('map.jpg');          % Load a sample image (floor)
xImage = [-0.4 0.4; -0.4 0.4];      % The x data for the image corners
yImage = [0.4 0.4; -0.4 -0.4];          % The y data for the image corners
zImage = [0.005 0.005; 0.005 0.005];% The z data for the image corners
surf(xImage,yImage,zImage,...       % Plot the surface
    'CData',img,...
    'FaceColor','texturemap');
hold on
end
%% Mapping
for i=1
    numSmallCell = 9;
    
    side_1_start = [0.270 -0.35 zOffset];
    side_1_end = [-0.270 -0.35 zOffset];
    side_1_yOffset = -0.35;
    
    side_2_start = [-0.35 -0.270 zOffset];
    side_2_end = [-0.35 0.270 zOffset];
    side_2_xOffset = -0.35;
    
    side_3_start = [-0.270 0.35 zOffset];
    side_3_end = [0.270 0.35 zOffset];
    side_3_yOffset = 0.35;
    
    side_4_start = [0.35 0.270 zOffset];
    side_4_end = [0.35 -0.270 zOffset];
    side_4_xOffset = 0.35;
end
%% Side 1
for i=1
side_1_xOffset = linspace(side_1_start(1,1),side_1_end(1,1),numSmallCell);
%LINSPACE(X1, X2, N) generates N points between X1 and X2.
plot3(side_1_xOffset,side_1_yOffset,zOffset,'.','MarkerSize',markerSize);
end
%% Side 2
for i=1
side_2_yOffset = linspace(side_2_start(1,2),side_2_end(1,2),numSmallCell);
plot3(side_2_xOffset,side_2_yOffset,zOffset,'.','MarkerSize',markerSize);
end
%% Side 3
for i=1
side_3_xOffset = linspace(side_3_start(1,1),side_3_end(1,1),numSmallCell);
plot3(side_3_xOffset,side_3_yOffset,zOffset,'.','MarkerSize',markerSize);
end
%% Side 4
for i=1
side_4_yOffset = linspace(side_4_start(1,2),side_4_end(1,2),numSmallCell);
plot3(side_4_xOffset,side_4_yOffset,zOffset,'.','MarkerSize',markerSize);
end

%% Allocate each cell location
for i=1
cellLocation = zeros(40,3); %% preallocate 
% Cell 1 (Start)
cell{1} = [0.35 -0.35 zOffset];
plot3(cell{1}(1,1),cell{1}(1,2),cell{1}(1,3),'.','MarkerSize',markerSize);

% Cell 2 to 10 (side 1)
for i = 2:10
    cellLocation(i,:) = [side_1_xOffset(1,i-1),side_1_yOffset,zOffset]; % pass in x,y,z
    cell{i} = cellLocation(i,:);
end

% Cell 11 (Jail - corner cell on bottom left)
cell{11} = [-0.35 -0.35 zOffset];
plot3(cell{11}(1,1),cell{11}(1,2),cell{11}(1,3),'.','MarkerSize',markerSize);

% Cell 12 to 20 (side 2)
for i = 12:20
    cellLocation(i,:) = [side_2_xOffset,side_2_yOffset(1,i-11),zOffset]; % pass in x,y,z
    cell{i} = cellLocation(i,:);
end

% Cell 21 l(Red car - corner cell on top left)
cell{21} = [-0.35 0.35 zOffset];
plot3(cell{21}(1,1),cell{21}(1,2),cell{21}(1,3),'.','MarkerSize',markerSize);

% Cell 22 to 30 (side 3)
for i = 22:30
    cellLocation(i,:) = [side_3_xOffset(1,i-21),side_3_yOffset,zOffset]; % pass in x,y,z
    cell{i} = cellLocation(i,:);
end

% Cell 31 (Arrest - corner cell on top right)
cell{31} = [0.35 0.35 zOffset];
plot3(cell{31}(1,1),cell{31}(1,2),cell{31}(1,3),'.','MarkerSize',markerSize);

% Cell 32 to 40 (side 4)
for i = 32:40
    cellLocation(i,:) = [side_4_xOffset,side_4_yOffset(1,i-31),zOffset]; % pass in x,y,z
    cell{i} = cellLocation(i,:);
end

% Cell 41 = cell 1 (start over)
cell{41} = cell{1};
end