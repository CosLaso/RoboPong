%% Movement of Balls [Lab 4.1] (Balls moves triangularly not projectile)

mesh_h = PlaceObject('PingPongRedBall.ply', [-0.95,0.25,1.445]);  % Bouncing ball (red)
vertices = get(mesh_h,'vertices');
mesh_h2 = PlaceObject('PingPongYellowBall.ply', [-0.95,-0.25,1.445]);   % Bouncing ball (yellow)
vertices2 = get(mesh_h2,'vertices');

i = 0;    % j represents x coordinate & i represents z coordinate (i is 1.445 globally)
k = 0;    % k is a condtional variable that triggers the bounce

for j = 0:0.05:1.5
    tr = transl(j,0,i);
    transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
    transformedVertices2 = [vertices2,ones(size(vertices2,1),1)] * tr';
    set(mesh_h,'vertices',transformedVertices(:,1:3));
    set(mesh_h2,'vertices',transformedVertices2(:,1:3));
    drawnow();
    pause(0.25);
    if (i <= -0.3)
        k = 1;
    end
    if (k == 1)
        i = (i+0.015);
    else
        i = (i-0.015);
    end
end