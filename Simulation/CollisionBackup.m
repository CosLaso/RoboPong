
% Stuff in the robot creation for purpose of visualisation
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

qMatrix = jtraj(q1,q2,steps);
result = true(steps,1);                 % A matrix of steps [rows] x 1 [column]

for i = 1:steps
    result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,false);
    robot.animate(qMatrix(i,:));
end

% Manually create cartesian waypoints - For a manipulator with fewer than 6DOF a mask matrix argument must be specified
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