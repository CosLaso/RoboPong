function [] = Red(robot)

    mesh_h = PlaceObject('BallRed.ply', [-0.95,0.3,1.445]);  % Bouncing ball (red)
    mesh_h2 = PlaceObject('BallYellow.ply', [-0.95,-0.3,1.445]);   % Bouncing ball (yellow)
    vertices = get(mesh_h,'vertices');
    vertices2 = get(mesh_h2,'vertices');
    
    redCatchPoint = [0.48,0.275,1.3];

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
    
    RMRCMovement(robot,redCatchPoint);
    PlaceObject('CupNew.ply',[redCatchPoint(1)+0.055 redCatchPoint(2)+0.03 redCatchPoint(3)-0.125]);
    
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
            i = (i+0.0075);
        else
            i = (i-0.02);
        end
    end

end
