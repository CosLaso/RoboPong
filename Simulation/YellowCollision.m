function [] = YellowCollision(robot)

    % Balls obtained from (https://free3d.com/3d-model/golf-ball-v1--411104.html)
    mesh_h = PlaceObject('BallRed.ply', [-0.95,0.3,1.445]);  % Bouncing ball (red)
    vertices = get(mesh_h,'vertices');
    mesh_h2 = PlaceObject('BallYellow.ply', [-0.95,-0.3,1.445]);   % Bouncing ball (yellow)
    vertices2 = get(mesh_h2,'vertices');

    yellowCatchPoint = [0.61,-0.275,1.3];
     
    i = 0;    % i represents z coordinate (i is 1.445 globally)
    m = 0;    % k is a condtional variable that triggers the bounce
    
    % Object to collide with parameters
    centerpnt = [0.55,-0.2,1.145];
    side = 0.2;
    plotOptions.plotFaces = true;
    
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

    [v,f,fn] = RectangularPrism(centerpnt-side/2, centerpnt+side/2, plotOptions);              % Collide for Yellow
    RMRCMovementWithCollision(robot,yellowCatchPoint,v,f,fn);
    PlaceObject('CupNew.ply',[yellowCatchPoint(1)-0.075 yellowCatchPoint(2)-0.03 yellowCatchPoint(3)-0.125]);

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

