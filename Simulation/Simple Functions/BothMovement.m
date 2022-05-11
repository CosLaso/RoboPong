%% Movement of Balls & Cup [Lab 4.1] - Need to adjust as measurements are slightly off

mesh_h = PlaceObject('PingPongRedBall.ply', [-0.95,0.25,1.445]);  % Bouncing ball (red)
vertices = get(mesh_h,'vertices');
mesh_h2 = PlaceObject('PingPongYellowBall.ply', [-0.95,-0.25,1.445]);   % Bouncing ball (yellow)
vertices2 = get(mesh_h2,'vertices');
mesh_h3 = PlaceObject('Cup.ply');
vertices3 = get(mesh_h3,'vertices');

BallColour = 0;

j = 0;    % j represents x cooridnate
i = 0;    % i represents z coordinate (i is 1.445 globally)
m = 0;    % k is a condtional variable that triggers the bounce

for k = -pi:0.1:-pi/2
    tr = transl(j,0,i);
    transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
    transformedVertices2 = [vertices2,ones(size(vertices2,1),1)] * tr';
    set(mesh_h,'vertices',transformedVertices(:,1:3));
    set(mesh_h2,'vertices',transformedVertices2(:,1:3));
    drawnow();
    pause(0.01);
    if (i <= -0.3)
        m = 1;
    end
    j = (j+0.1);
    if (m == 1)
        i = (i+0.025);
    else
        i = (i-0.03);
    end
    if (BallColour == 0)            % For red ball
        robot.animate([k,0,0,0]);
        tr2 = robot.fkine([k,0,0,0]);
        transformedVertices3 = [vertices3,ones(size(vertices3,1),1)] * tr2';
        set(mesh_h3,'vertices',transformedVertices3(:,1:3));
        drawnow();
        pause(0.01)
    else                            % For yellow ball
        robot.animate([k,0,0,0])
        tr2 = robot.fkine([k,0,0,0]);
        transformedVertices3 = [vertices3,ones(size(vertices3,1),1)] * tr2';
        set(mesh_h3,'vertices',transformedVertices3(:,1:3));
        drawnow();
        pause(0.01);
    end
end
