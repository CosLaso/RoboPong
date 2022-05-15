
%% Lab Assignment 2
% Cosmino Lasorsa - 13193228
% Ivy Qian Lei Ou - 13220052
% Mouloud Brarti - 13190475

close all;
clear all;
set(0,'DefaultFigureWindowStyle','docked');
clc
clf

%% Building 3D Environment

hold on

% % Place in Concrete Floor
% surf([-6,-6 ; 10,10], [-6,6 ; -6,6], [0,0 ; 0,0],'CData',imread('Concrete.jpg'),'FaceColor','texturemap');            % From UTS Canvas
% surf([-4,-4 ; 8,8], [-4,4 ; -4,4], [0,0 ; 0,0],'CData',imread('Tile.jpg'),'FaceColor','texturemap'); 

% % Place in Hazard Tape (from https://nationalsafetysigns.com.au/safety-signs/reflective-tape-sticker-hazard-tape-v2628/)
% surf([-2,-2;2,2],[-1.5,1.5;-1.5,1.5],[0,0;0,0],'CData',imread('Tape.jpg'),'FaceColor','texturemap');

% Placement of Table (from https://free3d.com/3d-model/straight-leg-coffee-tablewhite-v1--558417.html)
% PlaceObject('Table.ply',[0,0,0]);

% % Placement of Fence (from https://free3d.com/3d-model/fence-43609.html)
% PlaceObject('Fence1.ply', [-4, 2,1.7]);
% PlaceObject('Fence1.ply', [-4,-2,1.7]);
% PlaceObject('Fence1.ply', [ 8, 2,1.7]);
% PlaceObject('Fence1.ply', [ 8,-2,1.7]);
% PlaceObject('Fence2.ply', [ 2,-4,1.7]);
% PlaceObject('Fence2.ply', [-2,-4,1.7]);
% PlaceObject('Fence2.ply', [ 6,-4,1.7]);
% PlaceObject('Fence2.ply', [ 6, 4,1.7]);
% PlaceObject('Fence2.ply', [-2, 4,1.7]);

% % Placement of Emergency Stop Button (from https://free3d.com/3d-model/emergency-stop-button-813870.html)
% PlaceObject('Emergency_Stop.ply',[3,-3.8,1]);
% PlaceObject('Emergency_Stop.ply',[-3,-3.8,1]);

% % Placement of Fire Extinguisher (from https://free3d.com/3d-model/-fire-extinguisher-v3--639064.html)
% PlaceObject('Fire_Extinguisher.ply',[-3.8,3.25,0.55]);

% % Placement of Worker (from https://www.cgtrader.com/items/889520/download-page)
% PlaceObject('Worker.ply',[-1,-1.75,0]);
% PlaceObject('Worker2.ply', [2.2, 0, 0]);

% % Placement of Trash Can (from https://free3d.com/3d-model/rubbish-bin-83371.html)
% PlaceObject('Bin.ply',[-3.8,2.5,0]);

% % Placement of Plants (from ???)
% PlaceObject('Plants.ply', [6, 4.5, 0])
% PlaceObject('Plants.ply', [-2, 4.5, 0])

% % Placement of Sink (https://www.cgtrader.com/items/948227/download-page)
% PlaceObject('Sink.ply', [6, -3.8, 0])

% % Placement of Storage Container (from https://free3d.com/3d-model/storage-container-v2--782422.html)
% PlaceObject('Storage.ply',[-3.5,0,0]);
% PlaceObject('Storage.ply',[-3.5,-1.25,0]);
% PlaceObject('Storage.ply',[7.5,0,0]);
% PlaceObject('Storage.ply',[7.5,-1.25,0]);
% PlaceObject('Storage.ply',[7.5,1.25,0]);

% % Placement of Stool (from https://free3d.com/3d-model/wood-stool-303532.html)
% PlaceObject('Stool.ply',[0,-3.3,0]);
% PlaceObject('Stool.ply',[0.75,-3.3,0]);
% PlaceObject('Stool.ply',[1.65,-3.3,0]);
% PlaceObject('Stool.ply',[-3.65,1.5,0]);
% PlaceObject('Stool.ply',[-3.25,1.1,0]);
% PlaceObject('Stool.ply',[-3.5,-2.2,0]);

%% Simulate 4 DoF Robot (For Testing)

% L1 = Link('d',0,'a',0.0675,'alpha',0,'qlim',[-pi pi]);
% L2 = Link('d',0,'a',0.0675,'alpha',0,'qlim',[-pi pi]);
% L3 = Link('d',0,'a',0.0675,'alpha',0,'qlim',[-pi pi]);
% L4 = Link('d',0,'a',0.0675,'alpha',0,'qlim',[-pi pi]);
% robot = SerialLink([L1 L2 L3 L4],'name','myRobot');   
 
% robot.base = transl(0.55,0,1.145) * trotx(0,'deg') * troty(0,'deg');            % Rotate robot to the correct orientation
% robot.plot([-pi,0,0,0]);                                                        % Plot the robot into the environment

%% Simulate Dobot

% DobotBase = [0.55,0,1.145];

% Dobot = Dobot();
% Dobot = Dobot(transl(DobotBase));
% DobotWS = DobotWithoutSuction();
% DobotWS = DobotWithoutSuction(transl(DobotBase));

%% Movement of Balls & Cup & Simple Robotic Arm [Lab 4.1]

% % Balls obtained from (https://free3d.com/3d-model/golf-ball-v1--411104.html)
% mesh_h = PlaceObject('BallRed.ply', [-0.95,0.245,1.445]);  % Bouncing ball (red)
% vertices = get(mesh_h,'vertices');
% mesh_h2 = PlaceObject('BallYellow.ply', [-0.95,-0.245,1.445]);   % Bouncing ball (yellow)
% vertices2 = get(mesh_h2,'vertices');
% mesh_h3 = PlaceObject('CupFlipped.ply');
% vertices3 = get(mesh_h3,'vertices');
% mesh_h4 = PlaceObject('Cup.ply');
% vertices4 = get(mesh_h4,'vertices');
 
% BallColour = 0;
 
% j = 0;    % j represents x cooridnate
% i = 0;    % i represents z coordinate (i is 1.445 globally)
% m = 0;    % k is a condtional variable that triggers the bounce

% for k = 0:0.05:pi/2
%     tr = transl(j,0,i);
%     transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
%     transformedVertices2 = [vertices2,ones(size(vertices2,1),1)] * tr';
%     set(mesh_h,'vertices',transformedVertices(:,1:3));
%     set(mesh_h2,'vertices',transformedVertices2(:,1:3));
%     drawnow();
%     pause(0.01);
%     if (i <= -0.3)
%         m = 1;
%     end
%     j = (j+0.045);
%     if (m == 1)
%         i = (i+0.01);
%     else
%         i = (i-0.015);
%     end
%     if (BallColour == 0)            % For yellow ball
%         DobotWS.model.animate([k,0,0]);
%         tr2 = DobotWS.model.fkine([k,0,0]);
%         transformedVertices3 = [vertices3,ones(size(vertices3,1),1)] * tr2';
%         set(mesh_h3,'vertices',transformedVertices3(:,1:3));
%         drawnow();
%         pause(0.01)
%     else                            % For red ball
%         DobotWS.model.animate([-k,0,0])
%         tr2 = DobotWS.model.fkine([-k,0,0]);
%         transformedVertices4 = [vertices4,ones(size(vertices4,1),1)] * tr2';
%         set(mesh_h4,'vertices',transformedVertices4(:,1:3));
%         drawnow();
%         pause(0.01);
%     end
% end

%% RMRC (Resolved Rate Motion Control) [Lab 6] - Need to fix zooming issue

% testPoint = [0.55,0.245,1.2];
% RMRCMovement(DobotWS,testPoint);

%% Collision Checking/Avoidance [Lab 5] - No avoidance, just stopping

% % Creating an object to collide with
% centerpnt = [0.2,0,1.145];
% side = 0.2;
% plotOptions.plotFaces = true;
% [v,f,fn] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

% % Creating trajectory
% q1 = [pi/3,0,0]; 
% q2 = [-pi/3,0,0];
% steps = 50;
% qMatrix = jtraj(q1,q2,steps);

% % Finds if there are any collision on the robot path (using planes)
% for i = 1:steps
%     result = IsCollision(DobotWS,qMatrix(i,:),f,v,fn);
%     qMatrix(i,:)
%     DobotWS.model.animate(qMatrix(i,:));
%     drawnow();
%     if result == 1
%         error('Collision detected!')
%     end 
% end 
 
% % Finds if there are any collisions on the robot path (using points)
% testPoint = [0.55,0,1.145];
% collision = PointInCollision(DobotWS,q1,testPoint);
% if collision == 1
%     error('Collision detected!!');
% end

%% GUI (Graphical User Interface) [Subject Resources]

% See the app designer file in the directory and adapt it

%% IBVS (Image Based Visual Servoing) [Lab 10]

% Definitions
% Create image target (points in the image plane) 
pStar = [662 362 362 662; 362 362 662 662];

%Create 3D points
P = [1.8,1.8,1.8,1.8; -0.25,0.25,0.25,-0.25; 1.25,1.25,0.75,0.75];

% Make the Dobot
% Dobot = Dobot();
DobotWS = DobotWithoutSuction();          

%Initial pose
q0 = [pi/2, -pi/6, 0];

% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, 'resolution', [1024 1024], 'centre', [512 512],'name', 'UR10camera');

% frame rate
fps = 25;

% Define values
% Gain of the controler
lambda = 0.6;

% Depth of the IBVS
depth = mean (P(1,:));

% Initialise Simulation (Display in 3D)
% Display Dobot
Tc0= DobotWS.model.fkine(q0);
DobotWS.model.animate(q0);
drawnow();

% Plot camera and points
cam.T = Tc0;

% Display points in 3D and the camera
cam.plot_camera('Tcam',Tc0, 'label','scale',0.075);
plot_sphere(P, 0.025, 'b')
lighting gouraud
light

% Initialise Simulation (Display in Image view)
% Project points to the image
p = cam.plot(P, 'Tcam', Tc0);

% Camera view and plotting
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(P);    % show initial view

% Initialise display arrays
vel_p = [];
uv_p = [];
history = [];

% Loop of the visual servoing
ksteps = 0;
while true
    ksteps = ksteps + 1;

    % Compute the view of the camera
    uv = cam.plot(P);

    % Compute image plane error as a column
    e = pStar-uv;   % feature error
    e = e(:);
    Zest = [];

    % Compute the Jacobian
    if isempty(depth)
        % Exact depth from simulation (not possible in practice)
        pt = homtrans(inv(Tcam), P);
        J = cam.visjac_p(uv, pt(3,:) );
    elseif ~isempty(Zest)
        J = cam.visjac_p(uv, Zest);
    else
        J = cam.visjac_p(uv, depth );
    end

    % Compute the velocity of camera in camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        status = -1;
        return
    end
    fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

    % Compute robot's Jacobian and inverse
    J2 = DobotWS.model.jacobn(q0);
    Jinv = pinv(J2);

    % Get joint velocities
    qp = Jinv*v;

     % Maximum angular velocity cannot exceed 180 degrees/s
     ind=find(qp>pi);
     if ~isempty(ind)
         qp(ind)=pi;
     end
     ind=find(qp<-pi);
     if ~isempty(ind)
         qp(ind)=-pi;
     end

    % Update joints 
    q = q0 + (1/fps)*qp;
    DobotWS.model.animate(q);

    % Get camera location
%     Tc = DobotWS.model.fkine(q);
    Tc = DobotWS.model.fkine(DobotWS.model.getpos());
    cam.T = Tc;
    drawnow();

    % Update the history variables
    hist.uv = uv(:);
    vel = v;
    hist.vel = vel;
    hist.e = e;
    hist.en = norm(e);
    hist.jcond = cond(J);
    hist.Tcam = Tc;
    hist.vel_p = vel;
    hist.uv_p = uv;
    hist.qp = qp;
    hist.q = q;
    history = [history hist];
     pause(1/fps)
    if ~isempty(200) && (ksteps > 200)
        break;
    end

    % Update current joint position
    q0 = q;
end