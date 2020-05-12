%% Plot map
for i=1
img = imread('map.jpg');          % Load a sample image (floor)
xImage = [-0.4 0.4; -0.4 0.4];      % The x data for the image corners
yImage = [0.4 0.4; -0.4 -0.4];          % The y data for the image corners
% zImage = [0.005 0.005; 0.005 0.005];% The z data for the image corners
zImage = [0 0; 0 0];
surf(xImage,yImage,zImage,...       % Plot the surface
    'CData',img,...
    'FaceColor','texturemap');
hold on
end

%% Mapping
for i=1
    numSmallCell = 9;
    
    side_1_start = [0.270 -0.35 0];
    side_1_end = [-0.270 -0.35 0];
    side_1_yOffset = -0.35;
    
    side_2_start = [-0.35 -0.270 0];
    side_2_end = [-0.35 0.270 0];
    side_2_xOffset = -0.35;
    
    side_3_start = [-0.270 0.35 0];
    side_3_end = [0.270 0.35 0];
    side_3_yOffset = 0.35;
    
    side_4_start = [0.35 0.270 0];
    side_4_end = [0.35 -0.270 0];
    side_4_xOffset = 0.35;
end
%% Side 1
for i=1
side_1_zOffset = 0;
side_1_xOffset = linspace(side_1_start(1,1),side_1_end(1,1),numSmallCell);
%LINSPACE(X1, X2, N) generates N points between X1 and X2.
plot3(side_1_xOffset,side_1_yOffset,side_1_zOffset,'.','MarkerSize',50);
end
%% Side 2
for i=1
side_2_zOffset = 0;
side_2_yOffset = linspace(side_2_start(1,2),side_2_end(1,2),numSmallCell);
plot3(side_2_xOffset,side_2_yOffset,side_2_zOffset,'.','MarkerSize',50);
end
%% Side 3
for i=1
side_3_zOffset = 0;
side_3_xOffset = linspace(side_3_start(1,1),side_3_end(1,1),numSmallCell);
plot3(side_3_xOffset,side_3_yOffset,side_3_zOffset,'.','MarkerSize',50);
end
%% Side 4
for i=1
side_4_zOffset = 0;
side_4_yOffset = linspace(side_4_start(1,2),side_4_end(1,2),numSmallCell);
plot3(side_4_xOffset,side_4_yOffset,side_4_zOffset,'.','MarkerSize',50);
end

%% Allocate each cell location
cellLocation = zeros(40,3); %% preallocate 

% Cell 1 to 9
for i = 1:9
    cellLocation(i,:) = [side_1_xOffset(1,i),side_1_yOffset,side_1_zOffset]; % pass in x,y,z
    cell{i} = cellLocation(i,:);
end

% define cell 10 location here (Jail - big cell on bottom left)

% Cell 11 to 19
for i = 11:19
    cellLocation(i,:) = [side_2_xOffset,side_2_yOffset(1,i-10),side_2_zOffset]; % pass in x,y,z
    cell{i} = cellLocation(i,:);
end

% define cell 20 location here (Red car - big cell on top left)

% Cell 21 to 29
for i = 21:29
    cellLocation(i,:) = [side_3_xOffset(1,i-20),side_3_yOffset,side_3_zOffset]; % pass in x,y,z
    cell{i} = cellLocation(i,:);
end

% define cell 30 location here (Arrest - big cell on top right)

% Cell 31 to 39
for i = 31:39
    cellLocation(i,:) = [side_4_xOffset,side_4_yOffset(1,i-30),side_4_zOffset]; % pass in x,y,z
    cell{i} = cellLocation(i,:);
end

% cell 40 location = cell 0 %