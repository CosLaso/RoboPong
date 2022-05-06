%% Robotics
% Lab 5 - Questions 2 and 3: 3-link plannar collision check and avoidance
function [  ] = Collision_Avoidance_Test( )

% clf
close all;

% Make a 4DOF model
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);   
L4 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
robot = SerialLink([L1 L2 L3 L4],'name','myRobot');                     
q = zeros(1,4);                                                     % Create a vector of initial joint angles        
scale = 0.5;
workspace = [-2 2 -2 2 -0.05 2];                                    % Set the size of the workspace when drawing the robot
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
        
% Creating an object to collide with
centerpnt = [4,0,-0.5];
side = 1.5;
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
    result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,false);
    robot.animate(qMatrix(i,:));
end

%% Question 3

% clf
close all;

% Make a 4DOF model
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);   
L4 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
robot = SerialLink([L1 L2 L3 L4],'name','myRobot');                     
q = zeros(1,4);                                                     % Create a vector of initial joint angles        
scale = 0.5;
workspace = [-4 4 -4 4 -0.05 2];                                    % Set the size of the workspace when drawing the robot
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot

% Creating an object to collide with
centerpnt = [3,0,-0.5];
side = 1.5;
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

% 3.2: Manually create cartesian waypoints - For a manipulator with fewer than 6DOF a mask matrix argument must be specified
robot.animate(q1);
qWaypoints = [q1 ; robot.ikcon(transl(2.5,-1.1,0),q1)];
qWaypoints = [qWaypoints; robot.ikcon(transl(2,-1.1,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; robot.ikcon(transl(2.1,-1,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; robot.ikcon(transl(2.1,0,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; robot.ikcon(transl(2.1,1,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; robot.ikcon(transl(2.1,1.1,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; robot.ikcon(transl(2.5,1.1,0),q2)];
qWaypoints = [qWaypoints; q2];
qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
if IsCollision(robot,qMatrix,faces,vertex,faceNormals)
    error('Collision detected!!');
else
    disp('No collision found');
end
robot.animate(qMatrix);        

% 3.3: Randomly select waypoints (primative RRT)
robot.animate(q1);
qWaypoints = [q1;q2];
isCollision = true;
checkedTillWaypoint = 1;
qMatrix = [];
while (isCollision)
    startWaypoint = checkedTillWaypoint;
    for i = startWaypoint:size(qWaypoints,1)-1
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
        if ~IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals)
            qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
            robot.animate(qMatrixJoin);
            size(qMatrix)
            isCollision = false;
            checkedTillWaypoint = i+1;
            % Now try and join to the final goal (q2)
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
            if ~IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals)
                qMatrix = [qMatrix;qMatrixJoin];
                % Reached goal without collision, so break out
                break;
            end
        else
            % Randomly pick a pose that is not in collision
            qRand = (2 * rand(1,3) - 1) * pi;
            while IsCollision(robot,qRand,faces,vertex,faceNormals)
                qRand = (2 * rand(1,3) - 1) * pi;
            end
            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
            isCollision = true;
            break;
        end
    end
end
robot.animate(qMatrix)
keyboard

end


