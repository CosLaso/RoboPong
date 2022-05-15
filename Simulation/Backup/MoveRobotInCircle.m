%% Movement of Robot in a circle (with cup)

mesh_h3 = PlaceObject('Cup.ply');
vertices3 = get(mesh_h3,'vertices');

for i = 0:0.01:pi/2
    Dobot.model.animate([-i,0,0,0]);
    tr = Dobot.model.fkine([-i,0,0,0]);
    transformedVertices3 = [vertices3,ones(size(vertices3,1),1)] * tr';
    set(mesh_h3,'vertices',transformedVertices3(:,1:3));
    drawnow();
    pause(0.01);
end

for i = 0:0.01:pi/2
    Dobot.model.animate([-pi/2,-i,0,0]);
    tr = Dobot.model.fkine([-pi/2,-i,0,0]);
    transformedVertices3 = [vertices3,ones(size(vertices3,1),1)] * tr';
    set(mesh_h3,'vertices',transformedVertices3(:,1:3));
    drawnow();
    pause(0.01);
end

for i = pi/2:0.01:pi
    Dobot.model.animate([-pi/2,-i,0,0]);
    tr = Dobot.model.fkine([-pi/2,-i,0,0]);
    transformedVertices3 = [vertices3,ones(size(vertices3,1),1)] * tr';
    set(mesh_h3,'vertices',transformedVertices3(:,1:3));
    drawnow();
    pause(0.01);
end

for i = pi/2:0.01:pi
    Dobot.model.animate([-i,-pi,0,0]);
    tr = Dobot.model.fkine([-i,-pi,0,0]);
    transformedVertices3 = [vertices3,ones(size(vertices3,1),1)] * tr';
    set(mesh_h3,'vertices',transformedVertices3(:,1:3));
    drawnow();
    pause(0.01);
end
