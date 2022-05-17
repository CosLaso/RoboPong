%% Movement of Balls, Cup [Lab 4.1] & Simple Robotic Arm (with RMRC) [Lab 6] integrated with Collison Checking [Lab 5]

% Balls obtained from (https://free3d.com/3d-model/golf-ball-v1--411104.html)
mesh_h = PlaceObject('BallRed.ply', [-0.95,0.245,1.445]);  % Bouncing ball (red)
vertices = get(mesh_h,'vertices');
mesh_h2 = PlaceObject('BallYellow.ply', [-0.95,-0.245,1.445]);   % Bouncing ball (yellow)
vertices2 = get(mesh_h2,'vertices');
redCatchPoint = [0.55,0.245,1.2];
yellowCatchPoint = [0.55,-0.245,1.2];
 
Yellow = 0;
Red = 1;
BallColour = Yellow;
 
i = 0;    % i represents z coordinate (i is 1.445 globally)
m = 0;    % k is a condtional variable that triggers the bounce

% Creating an object to collide with
centerpnt = [0.55,-0.2,1.145];
centerpnt2 = [0.55,0.2,1.145];
side = 0.2;
plotOptions.plotFaces = true;
[v,f,fn] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);                % Collide for Yellow
% [v2,f2,fn2] = RectangularPrism(centerpnt2-side/2, centerpnt2+side/2,plotOptions);         % Collide for Red

for j = 0:0.075:0.15            % j represents x cooridnate
    tr = transl(j,0,i);
    transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
    transformedVertices2 = [vertices2,ones(size(vertices2,1),1)] * tr';
    set(mesh_h,'vertices',transformedVertices(:,1:3));
    set(mesh_h2,'vertices',transformedVertices2(:,1:3));
    drawnow();
    pause(0.25);
    i = (i-0.02);           % No bouncing till later so no if 
end

if (BallColour == 0)            % For yellow ball
    RMRCMovement(DobotWS,yellowCatchPoint,v,f,fn);
    PlaceObject('CupNew.ply',[yellowCatchPoint]);
else                            % For red ball
    RMRCMovement(DobotWS,redCatchPoint,v2,f2,fn2);
    PlaceObject('CupNew.ply',[redCatchPoint]);
end

for j = 0.15:0.075:1.45
    tr = transl(j,0,i);
    transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
    transformedVertices2 = [vertices2,ones(size(vertices2,1),1)] * tr';
    set(mesh_h,'vertices',transformedVertices(:,1:3));
    set(mesh_h2,'vertices',transformedVertices2(:,1:3));
    drawnow();
    pause(0.25);
    if (i <= -0.3)
        m = 1;
    end
    if (m == 1)
        i = (i+0.01);
    else
        i = (i-0.02);
    end
end