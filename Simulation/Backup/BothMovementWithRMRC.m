%% Movement of Balls & Cup & Simple Robotic Arm (with RMRC) [Lab 4.1] [Lab 6]

% Balls obtained from (https://free3d.com/3d-model/golf-ball-v1--411104.html)
mesh_h = PlaceObject('BallRed.ply', [-0.95,0.245,1.445]);  % Bouncing ball (red)
vertices = get(mesh_h,'vertices');
mesh_h2 = PlaceObject('BallYellow.ply', [-0.95,-0.245,1.445]);   % Bouncing ball (yellow)
vertices2 = get(mesh_h2,'vertices');
redCatchPoint = [0.55,0.245,1.2];
yellowCatchPoint = [0.55,-0.245,1.2];
 
BallColour = 1;
 
i = 0;    % i represents z coordinate (i is 1.445 globally)
m = 0;    % k is a condtional variable that triggers the bounce

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
    RMRCMovement(DobotWS,yellowCatchPoint);
    PlaceObject('CupNew.ply',[yellowCatchPoint])
else                            % For red ball
    RMRCMovement(DobotWS,redCatchPoint);
    PlaceObject('CupNew.ply',[redCatchPoint])
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