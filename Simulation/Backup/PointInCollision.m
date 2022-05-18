function collision = PointInCollision(robot,q,point)

    collision = 0;                         % Start as no collision before we check as we need to have an output

    ellipses = cell(1,4);                  % 5 joints + base
    ellipseParameters = zeros(5,3,4);      % 1st row is centres, 2nd row is radii, 3-5 is rotation matrix
    tr = zeros(4,4,robot.model.n+1);       % Create rotation matrix for each joint
    tr(:,:,1) = robot.base;                % First matrix is base
    L = robot.model.links;

    for i = 1:robot.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);    % Transform of joints
    end

    for i = 1:3

    % Common collision checking between joints
    centrePoint = [0,0,0];                         % Default centre point of joints
    z = (robot.model.links(i).d + robot.model.links(i).a);     % Distance in z direction determined by the DH parameters
    radii = [0.08,0.08,z];
    [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
    a = [x(:) y(:) z(:)];
    p1 = tr(1:3,4,i);                              % xyz of point from (coloumn 4 of matrix)
    p2 = tr(1:3,4,i+1);                            % xyz of point to
    d = [p2(1)-p1(1) p2(2)-p1(2) p2(3)-p1(3)];     % Distance between points
    c = p1+(d/2)';                                 % Midpoint of the points
    t = tr(:,:,i);                                      % The transform matrix of the current join
    xy = ((p2(1)-p1(1))^2 + (p2(2)-p1(2))^2)^0.5;       % Distance formula in the xy plane

        if i == 1 || i == 3                             % End joints
            t(1:3,1:3) = t(1:3,1:3);                    % No angle needed as the joints are parrallel to ground
            t(1:3,4) = c;                               % Set our translation joint to the midpoint calculated
            b = (t*[a,ones(size(a,1),1)]')';            % Changing centre point to be centre of the link ellipsoid
            ellipseParameters(1:2,:,i) = [c';radii];    % ???
            ellipseParameters(3:5,:,i) = eye(3);        % ???
            ellipses{i} = b(:,1:3);                     % ???
        else                                            % Middle joints
            angle = atan(((p2(3)-p2(3))/xy));
            t(1:3,1:3) = t(1:3,1:3) * roty(pi/2) * rotx(-angle);    % Need to obtain angle as they are not parrallel to ground
            t(1:3,4) = c;
            b = (t*[a, ones(size(a, 1), 1)]')';
            ellipseParameters(1:2,:,i) = [c';radii];
            ellipseParameters(3:5,:,i) = t(1:3,1:3) * roty(pi/2) * rotx(-angle);
            ellipses{i} = b(:,1:3);
        end
    end
    
    for i = 1:4                                      % Check collision for all joints (including base)
        t = transl(ellipseParameters(1,:,i)');                      % ???
        t(1:3,1:3) = ellipseParameters(3:5,:,i);                    % ???
        pointTr = [inv(t)*[point,ones(size(point,1),1)]']';         % Transform points
        updatedPoint = pointTr(:,1:3);                              % ???
        resetCentrePoint = [0 0 0];
        radii = [ellipseParameters(2,3,i) ellipseParameters(2,1,i) ellipseParameters(2,2,i)];       % Determine radii using updated parameters
        algebraicDistance = GetAlgebraicDistance(updatedPoint,resetCentrePoint,radii);              % See how far point is from reset centre point
        pointsInside = find(algebraicDistance < 1);     
        if pointsInside >= 1
            collision = 1;
        end
    end
end
