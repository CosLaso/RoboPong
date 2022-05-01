
%% Lab Assignment 2
% Cosmino Lasorsa - 13193228

close all;
clear all;
set(0,'DefaultFigureWindowStyle','docked')
clc
clf

%% Building 3D Environment

hold on

% % Place in Concrete Floor
% surf([-4,-4;4,4],[-4,4;-4,4],[0,0;0,0],'CData',imread('Concrete.jpg'),'FaceColor','texturemap');            % From UTS Canvas
% 
% % Place in Hazard Tape (from https://nationalsafetysigns.com.au/safety-signs/reflective-tape-sticker-hazard-tape-v2628/)
% surf([-2,-2;2,2],[-1.5,1.5;-1.5,1.5],[0,0;0,0],'CData',imread('Tape.jpg'),'FaceColor','texturemap');
% 
% % Placement of Table (from https://free3d.com/3d-model/straight-leg-coffee-tablewhite-v1--558417.html)
% PlaceObject('Table.ply',[0,0,0]);
% 
% % Placement of Fence (from https://free3d.com/3d-model/fence-43609.html) [2,4,1,3]
% PlaceObject('Fence.ply',[-4,0,1.4]);
% % PlaceObject('Fence.ply',[4,0,1.4]);
% % PlaceObject('FenceOD.ply',[0,4,1.4]);
% PlaceObject('FenceOD.ply',[0,-4,1.4]);          % FenceOD is the fence with xy axis flipped
% 
% % Placement of Emergency Stop Button (from https://free3d.com/3d-model/emergency-stop-button-813870.html)
% PlaceObject('Emergency_Stop.ply',[3,-3.8,1]);
% 
% % Placement of Fire Extinguisher (from https://free3d.com/3d-model/-fire-extinguisher-v3--639064.html)
% PlaceObject('Fire_Extinguisher.ply',[-3.8,3.25,0.55]);
% 
% % Placement of Worker (from https://www.cgtrader.com/items/889520/download-page)
% PlaceObject('Worker.ply',[-2,-2.75,0]);
% 
% % Placement of Trash Can (from https://free3d.com/3d-model/rubbish-bin-83371.html)
% PlaceObject('Bin.ply',[-3.8,2.5,0]);
% 
% % Placement of Storage Container (from https://free3d.com/3d-model/storage-container-v2--782422.html)
% PlaceObject('Storage.ply',[-3.5,0,0]);
% PlaceObject('Storage.ply',[-3.5,-1.25,0]);
% 
% % Placement of Stool (from https://free3d.com/3d-model/wood-stool-303532.html)
% PlaceObject('Stool.ply',[0,-3.5,0]);
% PlaceObject('Stool.ply',[0.75,-3.25,0]);
% PlaceObject('Stool.ply',[1.65,-3.3,0]);
% PlaceObject('Stool.ply',[-3.65,1.5,0]);
% PlaceObject('Stool.ply',[-3.25,1.1,0]);
% PlaceObject('Stool.ply',[-3.5,-2.2,0]);

%% Simulate Dobot ***

% % Offsets for the robotics arm is off at the moment
% Dobot = Dobot(false);

%% Movement of Robot [Lab 4.2] ***

% qiJointAngles = [0 0 0 0];
% 
% point = [0.15,0.1,0.9];
% % finish = [0.35,0.05,0.8];
% steps = 50;
% 
% % Go from start to end
% qtJointAngles = Dobot.model.ikine(point,qiJointAngles,[1 1 1 0 0 0]);
% jointTrajectory = jtraj(qiJointAngles,qtJointAngles,steps);
% for i = 1:steps
%     animate(Dobot.model,jointTrajectory(i,:));
%     drawnow();
%     pause(0.01)
% end

% function translateto(self)
%     steps = 100;
%     modelq_i = [0 0 0 0];
%     target = [-0.25, 0.25, 0.2];
%     position = trasnl(target)*trotx(pi);
%     modelikcon = self.model.ikcon(position,modelq_i);
%     Trajmodel = jtraj(modelq_i,modelikcon,steps);
%     Trajmodel (:,4) = 0;
%     for i = 1:steps
%         animate(self.model,Trajmodel(i,:));
%         drawnow();
%         pause(0.01);
%     end
% end

%% Movement of Balls [Lab 4.1]

% % Ball just moves triangularly not parabolic (projectile)
% mesh_h = PlaceObject('Ping_Pong_Ball.ply', [0,0,2]);
% vertices = get(mesh_h,'Vertices');
% 
% axis([-1,5,-1,5,-1,5]);
% 
% i = 0;    % j represents x coordinate & i represents z coordinate (i is 2 globally)
% k = 0;    % k is a condtional variable that triggers the bounce
% 
% for j = 0:0.2:4
%     tr = transl(j,0,i);
%     transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
%     set(mesh_h,'Vertices',transformedVertices(:,1:3));
%     drawnow();
%     pause(0.25);
%     if (i <= -2)
%         k = 1;
%     end
%     if (k == 0)
%         i = (i-0.2);
%     else
%         i = (i+0.2);
%     end
% end

%% Simulate 4 DoF Robot - use this until Dobot Models are solved

% L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
% L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
% L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]) 
% L4 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
% robot = SerialLink([L1 L2 L3 L4],'name','myRobot');   
% 
% % % Rotate the base around the Y axis so the Z axis faces downways
% % robot.base = troty(pi);

%% RMRC (Resolved Rate Motion Control) [Lab 6]

% % Need to adapt this to include 2 extra degrees of freedom
% 
% mdl_planar2;                                % Load 2-Link Planar Robot
% 
% T1 = [eye(3) [1.5 1 0]'; zeros(1,3) 1];       % First pose
% T2 = [eye(3) [1.5 -1 0]'; zeros(1,3) 1];      % Second pose
% steps = 50;
% 
% % 3.6
% M = [1 1 zeros(1,4)];                         % Masking Matrix
% x1 = [1.5 1]';
% x2 = [1.5 -1]';
% deltaT = 0.05;                                        % Discrete time step
% 
% % 3.7
% x = zeros(2,steps);
% s = lspb(0,1,steps);                                 % Create interpolation scalar
% for i = 1:steps
%     x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
% end
% 
% % 3.8
% qMatrix = nan(steps,2);
% 
% % 3.9
% qMatrix(1,:) = p2.ikine(T1,[0 0],M);                 % Solve for joint angles
% 
% % 3.10
% for i = 1:steps-1
%     xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
%     J = p2.jacob0(qMatrix(i,:));            % Get the Jacobian at the current state
%     J = J(1:2,:);                           % Take only first 2 rows
%     qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
%     qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                   % Update next joint state
% end
% 
% p2.plot(qMatrix,'trail','r-');


%% Collision Checking/Avoidance [Lab 5]***

% % Need to adapt this code once we have the Dobot model working correctly
% % 2.4: Get the transform of every joint (i.e. start and end of every link)
% tr = zeros(4,4,robot.n+1);
% tr(:,:,1) = robot.base;
% L = robot.links;
% for i = 1 : robot.n
%     tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
% end
% 
% % 2.5: Go through each link and also each triangle face
% for i = 1 : size(tr,3)-1    
%     for faceIndex = 1:size(faces,1)
%         vertOnPlane = vertex(faces(faceIndex,1)',:);
%         [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
%         if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
%             plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
%             display('Intersection');
%         end
%     end    
% end
% 
% % 2.6: Go through until there are no step sizes larger than 1 degree
% q1 = [-pi/4,0,0];
% q2 = [pi/4,0,0];
% steps = 2;
% while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
%     steps = steps + 1;
% end
% qMatrix = jtraj(q1,q2,steps);
% 
% % 2.7
% result = true(steps,1);
% for i = 1: steps
%     result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,false);
%     robot.animate(qMatrix(i,:));
% end
% 
% % 3.2: Manually create cartesian waypoints
% robot.animate(q1);
% qWaypoints = [q1 ; robot.ikcon(transl(1.5,-1,0),q1)];
% qWaypoints = [qWaypoints; robot.ikcon(transl(1,-1,0),qWaypoints(end,:))];
% qWaypoints = [qWaypoints; robot.ikcon(transl(1.1,-0.5,0),qWaypoints(end,:))];
% qWaypoints = [qWaypoints; robot.ikcon(transl(1.1,0,0),qWaypoints(end,:))];
% qWaypoints = [qWaypoints; robot.ikcon(transl(1.1,0.5,0),qWaypoints(end,:))];
% qWaypoints = [qWaypoints; robot.ikcon(transl(1.1,1,0),qWaypoints(end,:))];
% qWaypoints = [qWaypoints; robot.ikcon(transl(1.5,1,0),q2)];
% qWaypoints = [qWaypoints; q2];
% qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
% if IsCollision(robot,qMatrix,faces,vertex,faceNormals)
%     error('Collision detected!!');
% else
%     display('No collision found');
% end
% robot.animate(qMatrix);        
% 
% % 3.3: Randomly select waypoints (primative RRT)
% robot.animate(q1);
% qWaypoints = [q1;q2];
% isCollision = true;
% checkedTillWaypoint = 1;
% qMatrix = [];
% while (isCollision)
%     startWaypoint = checkedTillWaypoint;
%     for i = startWaypoint:size(qWaypoints,1)-1
%         qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
%         if ~IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals)
%             qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
%             robot.animate(qMatrixJoin);
%             size(qMatrix)
%             isCollision = false;
%             checkedTillWaypoint = i+1;
%             % Now try and join to the final goal (q2)
%             qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
%             if ~IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals)
%                 qMatrix = [qMatrix;qMatrixJoin];
%                 % Reached goal without collision, so break out
%                 break;
%             end
%         else
%             % Randomly pick a pose that is not in collision
%             qRand = (2 * rand(1,3) - 1) * pi;
%             while IsCollision(robot,qRand,faces,vertex,faceNormals)
%                 qRand = (2 * rand(1,3) - 1) * pi;
%             end
%             qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
%             isCollision = true;
%             break;
%         end
%     end
% end
% robot.animate(qMatrix)

%% GUI (Graphical User Interface) [Subject Resources]

% See the app designer file in the directory and adapt it

%% Visual Servoing [Lab 10]

