%% Movement of Cup (on End Effector) [Lab 4.1]

mesh_h3 = PlaceObject('Cup.ply');
vertices3 = get(mesh_h3,'vertices');

BallColour = 0;

if (BallColour == 0)            % For red ball
    for i = -pi:0.01:-pi/2
        robot.animate([i,0,0,0]);
        tr = robot.fkine([i,0,0,0]);
        transformedVertices3 = [vertices3,ones(size(vertices3,1),1)] * tr';
        set(mesh_h3,'vertices',transformedVertices3(:,1:3));
        drawnow();
        pause(0.01);
    end
else                            % For yellow ball
    for i = -pi:-0.01:(3*pi)/2
        robot.animate([i,0,0,0])
        tr = robot.fkine([i,0,0,0]);
        transformedVertices3 = [vertices3,ones(size(vertices3,1),1)] * tr';
        set(mesh_h3,'vertices',transformedVertices3(:,1:3));
        drawnow();
        pause(0.01);
    end    
end