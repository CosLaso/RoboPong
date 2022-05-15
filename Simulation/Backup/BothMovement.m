%% Movement of Balls & Cup [Lab 4.1]

mesh_h = PlaceObject('BallRed.ply', [-0.95,0.245,1.445]);  % Bouncing ball (red)
vertices = get(mesh_h,'vertices');
mesh_h2 = PlaceObject('Ball.ply', [-0.95,-0.245,1.445]);   % Bouncing ball (yellow)
vertices2 = get(mesh_h2,'vertices');
mesh_h3 = PlaceObject('CupFlipped.ply');
vertices3 = get(mesh_h3,'vertices');
mesh_h4 = PlaceObject('Cup.ply');
vertices4 = get(mesh_h4,'vertices');

BallColour = 0;

j = 0;    % j represents x cooridnate
i = 0;    % i represents z coordinate (i is 1.445 globally)
m = 0;    % k is a condtional variable that triggers the bounce

for k = 0:0.05:pi/2
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
    j = (j+0.045);
    if (m == 1)
        i = (i+0.01);
    else
        i = (i-0.015);
    end
    if (BallColour == 0)            % For yellow ball
        Dobot.model.animate([k,0,0,0]);
        tr2 = Dobot.model.fkine([k,0,0,0]);
        transformedVertices3 = [vertices3,ones(size(vertices3,1),1)] * tr2';
        set(mesh_h3,'vertices',transformedVertices3(:,1:3));
        drawnow();
        pause(0.01)
    else                            % For red ball
        Dobot.model.animate([-k,0,0,0])
        tr2 = Dobot.model.fkine([-k,0,0,0]);
        transformedVertices4 = [vertices4,ones(size(vertices4,1),1)] * tr2';
        set(mesh_h4,'vertices',transformedVertices4(:,1:3));
        drawnow();
        pause(0.01);
    end
end