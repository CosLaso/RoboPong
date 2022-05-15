
close all;
clear all;
set(0,'DefaultFigureWindowStyle','docked');
clc
clf

%% Simulate 4 DoF Robot (For Testing)

% L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
% L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
% L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
% L4 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
% robot = SerialLink([L1 L2 L3 L4],'name','myRobot');   
% 
% % robot.base = transl(0.55,0,1.145) * trotx(0,'deg') * troty(0,'deg');            % Rotate robot to the correct orientation
% robot.plot([-pi,0,0,0]);                                                        % Plot the robot into the environment

Dobot = Dobot(false);

%% Collision Stuff

% Stuff in the robot creation for purpose of visualisation
q = zeros(1,4);                                                     % Create a vector of initial joint angles        
scale = 0.25;
workspace = [-2 2 -2 2 -0.05 2];                                    % Set the size of the workspace when drawing the robot
Dobot.model.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot

% Creating an object to collide with
centerpnt = [1,0,-0.5];
side = 0.75;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal

% Set Moving Parameters
q1 = [-pi/4,0,0 0];
q2 = [pi/4,0,0 0];
steps = 2;

while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end

qMatrix = jtraj(q1,q2,steps);
result = true(steps,1);                 % A matrix of steps [rows] x 1 [column]

for i = 1:steps
    result(i) = IsCollision(Dobot.model,qMatrix(i,:),faces,vertex,faceNormals,false);
    Dobot.model.animate(qMatrix(i,:));
end

% Manually create cartesian waypoints - For a manipulator with fewer than 6DOF a mask matrix argument must be specified
Dobot.model.animate(q1);
qWaypoints = [q1 ; Dobot.model.ikcon(transl(2.5,-1.1,0),q1)];
qWaypoints = [qWaypoints; Dobot.model.ikcon(transl(2,-1.1,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; Dobot.model.ikcon(transl(2.1,-1,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; Dobot.model.ikcon(transl(2.1,0,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; Dobot.model.ikcon(transl(2.1,1,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; Dobot.model.ikcon(transl(2.1,1.1,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; Dobot.model.ikcon(transl(2.5,1.1,0),q2)];
qWaypoints = [qWaypoints; q2];
qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
if IsCollision(Dobot.model,qMatrix,faces,vertex,faceNormals)
    error('Collision detected!!');
else
    disp('No collision found');
end
Dobot.model.animate(qMatrix);  