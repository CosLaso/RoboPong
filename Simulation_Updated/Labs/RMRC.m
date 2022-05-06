%% Robotics
% Lab 6 
function [  ] = Lab6Solution( )
close all;

%% Questions 1:	Convert range measurements to a point cloud
clf
%% 1.1
robot = SchunkUTSv2_0();
q = [0,pi/2,0,0,0,0];
robot.plot3d(q);
view(3);
camlight;
hold on;
pause(3);

%% 1.2
% Plane params (to work out distances for questions)
createQuestion = true; % For creating the lab exercise question
if createQuestion
    pNormal = [-0.3090, 0.9511, 0];            % Create questions
    pPoint = [0,3,0];                          % Create questions
end
tr = robot.fkine(q)
startP = tr(1:3,4)';
if createQuestion
    endP = tr(1:3,4)' + 10 * tr(1:3,3)';                             % Create questions
    intersectP = LinePlaneIntersection(pNormal,pPoint,startP,endP);  % Create questions
    dist = dist2pts(startP,intersectP)                               % Create questions
else    
    dist = 1.9594;
end
endP = tr(1:3,4)' + dist * tr(1:3,3)';
line1_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
plot3(endP(1),endP(2),endP(3),'r*');
axis equal;
verts = endP;
pause(1);
% try delete(line1_h); end

%% 1.3
% New pose 1
q = [pi/10,pi/2,0,0,0,0];
tr = robot.fkine(q);
robot.animate(q);
startP = tr(1:3,4)';
if createQuestion
    endP = tr(1:3,4)' + 10 * tr(1:3,3)';                             % Create questions
    intersectP = LinePlaneIntersection(pNormal,pPoint,startP,endP);  % Create questions
    dist = dist2pts(startP,intersectP)                               % Create questions
else    
    dist = 2.4861;
end
endP = tr(1:3,4)' + dist * tr(1:3,3)';
line_h(2) = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
plot3(endP(1),endP(2),endP(3),'r*');
verts = [verts; endP];
pause(1);

% New pose 2
q = [-pi/10,5*pi/12,0,0,0,0];
tr = robot.fkine(q);
robot.animate(q);
startP = tr(1:3,4)';
if createQuestion
    endP = tr(1:3,4)' + 10 * tr(1:3,3)';                            % Create questions
    intersectP = LinePlaneIntersection(pNormal,pPoint,startP,endP); % Create questions
    dist = dist2pts(startP,intersectP)                              % Create questions
else
    dist = 1.9132;
end
endP = tr(1:3,4)' + dist * tr(1:3,3)';
line_h(3) = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r'); %#ok<NASGU>
plot3(endP(1),endP(2),endP(3),'r*');
verts = [verts; endP];
axis equal
pause(1);

%% 1.4
% Three points on the triangle are in verts
triangleNormal = cross((verts(1,:)-verts(2,:)),(verts(2,:)-verts(3,:)));
triangleNormal = triangleNormal / norm(triangleNormal);

% Make a plane at the orgin, to rotate later
basePlaneNormal = [-1,0,0];
[Y,Z] = meshgrid(-2:0.1:2,-2:0.1:2  );
sizeMat = size(Y);
X = repmat(0,sizeMat(1),sizeMat(2));

% Rotation axis: to rotate the base plane around
rotationAxis = cross(triangleNormal,basePlaneNormal);
rotationAxis = rotationAxis / norm(rotationAxis);

% Rotation angle: how much to rotate base plane to make it match triangle plane
rotationRadians = acos(dot(triangleNormal,basePlaneNormal));

% Make a transform to do that rotation
tr = makehgtform('axisrotate',rotationAxis,rotationRadians);

% Find a central point of the triangle
trianglePoint = sum(verts)/3;

% Plot the point/normal of the triangle
plot3(trianglePoint(1),trianglePoint(2),trianglePoint(3),'g*');
plot3([trianglePoint(1),trianglePoint(1)+triangleNormal(1)] ...
     ,[trianglePoint(2),trianglePoint(2)+triangleNormal(2)] ...
     ,[trianglePoint(3),trianglePoint(3)+triangleNormal(3)],'b');
 drawnow();
 pause(1);
 
% Transform the points on the default plane, to matches the actual triangle
points = [X(:),Y(:),Z(:)] * tr(1:3,1:3) + repmat(trianglePoint,prod(sizeMat),1);
X = reshape(points(:,1),sizeMat(1),sizeMat(2));
Y = reshape(points(:,2),sizeMat(1),sizeMat(2));
Z = reshape(points(:,3),sizeMat(1),sizeMat(2));

% Make points where Z<0 to be = zero
Z(Z<0)= 0;
surf(X,Y,Z);
pause(1);

%% 1.5 and 1.6
maxRange = 3; % meters

% get end-effector point in given pose
q = [0,pi/2,0,0,0,0];
tr = robot.fkine(q);
robot.animate(q);
startP = tr(1:3,4)';

% Make a single scan ray as if it were from the origin (for rotating)
rayAtOrigin = maxRange * tr(1:3,3)';
xRotAxis = tr(1:3,1)';
yRotAxis = tr(1:3,2)';

% Rotate around end effector's xaxis and yaxis, populate scanData
scanData = [];
for xRotRads = deg2rad(-20):deg2rad(1):deg2rad(20)
    for yRotRads = deg2rad(-20):deg2rad(1):deg2rad(20) % Note it is more efficient to make one scan block rather than having an embeded for loop as done here
        % Make a transform to rotate the scan ray around
        tr = makehgtform('axisrotate',xRotAxis,xRotRads) * makehgtform('axisrotate',yRotAxis,yRotRads);
        
        % Determine location of ray end at max range
        rayEnd = startP +  rayAtOrigin * tr(1:3,1:3);
        
        % Check for intersection with the plane that the triangle is on
        [intersectP,check] = LinePlaneIntersection(triangleNormal,trianglePoint,startP,rayEnd);        
        if check == 1
            rayEnd = intersectP;
        end
        
        % Check for intersection with floor (note: floorNormal = [0,0,1] floorPoint = origin)
        [intersectWithFloor,check] = LinePlaneIntersection([0,0,1],[0,0,0],startP,rayEnd);
        if check == 1 && dist2pts(startP,intersectWithFloor) < dist2pts(startP,rayEnd)
            rayEnd = intersectWithFloor;
        end
        
        % Keep track of the scan data. It is more efficient to initialise
        % it to the correct size, however for brevity, it is done suboptimally here
        scanData = [scanData; rayEnd];         %#ok<AGROW>
    end
end
% Plot the scan data
plot3(scanData(:,1),scanData(:,2),scanData(:,3),'r.');
keyboard

%% Question 2: Ellipsoid and Point collision checking
% 2.1
clf;
centerPoint = [0,0,0];
radii = [3,2,1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
view(3);
hold on;

% 2.2
ellipsoidAtOrigin_h = surf(X,Y,Z);
% Make the ellipsoid translucent (so we can see the inside and outside points)
alpha(0.1);

% 2.3
% One side of the cube
[Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
sizeMat = size(Y);
X = repmat(0.75,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);

% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];         
         
% Plot the cube's point cloud         
cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([2,0,-0.5],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
axis equal

% 2.4
algebraicDist = GetAlgebraicDist(cubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
display(['2.4: There are ', num2str(size(pointsInside,1)),' points inside']);

% 2.5
centerPoint = [1,1,1];
algebraicDist = GetAlgebraicDist(cubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
display(['2.5: There are now ', num2str(size(pointsInside,1)),' points inside']);

% 2.6
centerPoint = [0,0,0];
cubePointsAndOnes = [inv(transl(1,1,1)) * [cubePoints,ones(size(cubePoints,1),1)]']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
algebraicDist = GetAlgebraicDist(cubePoints, centerPoint, radii);          
pointsInside = find(algebraicDist < 1);
display(['2.6: There are now ', num2str(size(pointsInside,1)),' points inside']);

% 2.7
centerPoint = [0,0,0];
cubePointsAndOnes = [inv(transl(1,1,1)*trotx(pi/4)) * [cubePoints,ones(size(cubePoints,1),1)]']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
display(['2.7: There are now ', num2str(size(pointsInside,1)),' points inside']);
pause(1);

% 2.8
try delete(cubeAtOigin_h); end;
try delete(ellipsoidAtOrigin_h); end;
try delete(oneSideOfCube_h); end;

L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])        
robot = SerialLink([L1 L2 L3],'name','myRobot');  

% New values for the ellipsoid (guessed these, need proper model to work out correctly)
centerPoint = [0,0,0];
radii = [1,0.5,0.5];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:4
    robot.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.faces{i} = delaunay(robot.points{i});    
    warning on;
end

robot.plot3d([0,0,0]);
axis equal
camlight
% robot.teach
% keyboard

% 2.9
q = [0,0,0]
tr = robot.fkine(q);
cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
display(['2.9: There are now ', num2str(size(pointsInside,1)),' points inside']);

% 2.10
q = [0,0,0]
tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base;
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

% Go through each ellipsoid
for i = 1: size(tr,3)
    cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
    updatedCubePoints = cubePointsAndOnes(:,1:3);
    algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);
    display(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
end

keyboard

%% Quesiton 3
% Joint Interpolation

% 3.1
steps = 50;
mdl_planar2;                                  % Load 2-Link Planar Robot

% 3.2
T1 = [eye(3) [1.5 1 0]'; zeros(1,3) 1];       % First pose
T2 = [eye(3) [1.5 -1 0]'; zeros(1,3) 1];      % Second pose

% 3.3
M = [1 1 zeros(1,4)];                         % Masking Matrix
q1 = p2.ikine(T1,[0 0],M);                    % Solve for joint angles
q2 = p2.ikine(T2,[0 0],M);                    % Solve for joint angles
p2.plot(q1,'trail','r-');
pause(3)
% 3.4
qMatrix = jtraj(q1,q2,steps);
p2.plot(qMatrix,'trail','r-');

% 3.5: Resolved Motion Rate Control
steps = 50;

% 3.6
x1 = [1.5 1]';
x2 = [1.5 -1]';
deltaT = 0.05;                                        % Discrete time step

% 3.7
x = zeros(2,steps);
s = lspb(0,1,steps);                                 % Create interpolation scalar
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
end

% 3.8
qMatrix = nan(steps,2);

% 3.9
qMatrix(1,:) = p2.ikine(T1,[0 0],M);                 % Solve for joint angles

% 3.10
for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
    J = p2.jacob0(qMatrix(i,:));            % Get the Jacobian at the current state
    J = J(1:2,:);                           % Take only first 2 rows
    qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
    qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                   % Update next joint state
end

p2.plot(qMatrix,'trail','r-');

end


%% dist2pts
%
% *Description:*  Function for find the distance between 2 or the same number of 3D points

%% Function Call
% 
% *Inputs:* 
%
% _pt1_ (many*(2||3||6) double) x,y || x,y,z cartesian point ||Q Joint angle
%
% _pt2_ (many*(2||3||6) double) x,y || x,y,z cartesian point ||Q Joint angle
%
% *Returns:* 
%
% _dist_ (double) distance from pt1 to pt2

function dist=dist2pts(pt1,pt2)

%% Calculate distance (dist) between consecutive points
% If 2D
if size(pt1,2) == 2
    dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
              (pt1(:,2)-pt2(:,2)).^2);
% If 3D          
elseif size(pt1,2) == 3
    dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
              (pt1(:,2)-pt2(:,2)).^2+...
              (pt1(:,3)-pt2(:,3)).^2);
% If 6D like two poses
elseif size(pt1,2) == 6
    dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
              (pt1(:,2)-pt2(:,2)).^2+...
              (pt1(:,3)-pt2(:,3)).^2+...
              (pt1(:,4)-pt2(:,4)).^2+...
              (pt1(:,5)-pt2(:,5)).^2+...
              (pt1(:,6)-pt2(:,6)).^2);
end
end

%% GetAlgebraicDist
% determine the algebraic distance given a set of points and the center
% point and radii of an elipsoid
% *Inputs:* 
%
% _points_ (many*(2||3||6) double) x,y,z cartesian point
%
% _centerPoint_ (1 * 3 double) xc,yc,zc of an ellipsoid
%
% _radii_ (1 * 3 double) a,b,c of an ellipsoid
%
% *Returns:* 
%
% _algebraicDist_ (many*1 double) algebraic distance for the ellipsoid

function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end
