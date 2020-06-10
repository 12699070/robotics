%% Collision Detection
% Function obtained from Lab9 - Quiz 4 

% robot = Robot model
% qMatrix = Joint state vector (trajectory)
% faces,vertex,faceNormals = Triangles obstacle

function result = CollisionDetection(robot,qMatrix,brick,returnOnceFound)
if nargin < 4
    returnOnceFound = true;
end
% Initialize the result
result = false;

% Triangles obstacle 
faces = brick.f;
vertex = brick.updatedPoints(:,1:3);
faceNormals = [brick.data.vertex.nx,brick.data.vertex.ny,brick.data.vertex.nz];

for qIndex = 1:size(qMatrix,1)
    % Transforms of every joint of the robot model
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'r*');
%                 display('Intersection');n
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end

%% GetLinkPoses
function transforms = GetLinkPoses(q, robot)    % q = Robot joint angles | Robot = Robot model
% Links
links = robot.links;
% Transforms
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    currentTransform = transforms(:,:,i);
    currentTransform = currentTransform * trotz(q(1,i)+L.offset) * transl(0,0,L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = currentTransform;
end
end

%% LinePlaneIntersection
% Given a plane (normal and point) and two points that make up another line, get the intersection
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment
function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

intersectionPoint = [0 0 0];
u = point2OnLine - point1OnLine;
w = point1OnLine - pointOnPlane;
D = dot(planeNormal,u);
N = -dot(planeNormal,w);
check = 0; %#ok<NASGU>
if abs(D) < 10^-7       % The segment is parallel to plane
    if N == 0           % The segment lies in plane
        check = 2;
        return
    else
        check = 0;      % No intersection
        return
    end
end

% Compute the intersection parameter
sI = N / D;
intersectionPoint = point1OnLine + sI.*u;

if (sI < 0 || sI > 1)
    check= 3;          % The intersection point lies outside the segment, so there is no intersection
else
    check=1;
end
end


%% IsIntersectionPointInsideTriangle
% Check point inside the triangle at a distance lower than 0.05 (result = 1) or outside (result = 0)

function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

dist = 0.1;
if w < [dist,dist,dist]
    result = 1;                    % intersectP is in Triangle
    return;
end

result = 0;
end


