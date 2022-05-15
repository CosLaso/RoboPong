%% Collision Checking (No Avoidance - Just Stopping) [Lab 5]

% Creating an object to collide with
centerpnt = [0.2,0,1.145];
side = 0.2;
plotOptions.plotFaces = true;
[v,f,fn] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

% Creating trajectory
q1 = [pi/3,0,0]; 
q2 = [-pi/3,0,0];
steps = 50;
qMatrix = jtraj(q1,q2,steps);

% Finds if there are any collision on the robot path (using planes)
for i = 1:steps
    result = IsCollision(DobotWS,qMatrix(i,:),f,v,fn);
    qMatrix(i,:)
    DobotWS.model.animate(qMatrix(i,:));
    drawnow();
    if result == 1
        error('Collision detected!')
    end 
end 

% Finds if there are any collisions on the robot path (using points)
testPoint = [0.55,0,1.145];
collision = pointsInCollision(DobotWS,q1,testPoint);

if collision == 1
    error('Collision detected!!');
end