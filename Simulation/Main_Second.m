
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

% % Placement of Table (from https://free3d.com/3d-model/straight-leg-coffee-tablewhite-v1--558417.html)
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

%% Simulate Dobot

DobotBase = [0.55,0,1.145];

% Dobot = Dobot();
Dobot = Dobot(transl(DobotBase));
% DobotWS = DobotWithoutSuction();
% DobotWS = DobotWithoutSuction(transl(DobotBase));

%% Movement of Balls, Cup [Lab 4.1] & Simple Robotic Arm (with RMRC) [Lab 6] integrated with Collison Checking [Lab 5]

% % Balls obtained from (https://free3d.com/3d-model/golf-ball-v1--411104.html)
% mesh_h = PlaceObject('BallRed.ply', [-0.95,0.3,1.445]);  % Bouncing ball (red)
% vertices = get(mesh_h,'vertices');
% mesh_h2 = PlaceObject('BallYellow.ply', [-0.95,-0.3,1.445]);   % Bouncing ball (yellow)
% vertices2 = get(mesh_h2,'vertices');
% yellowCatchPoint = [0.61,-0.275,1.3];
% redCatchPoint = [0.48,0.275,1.3];
%  
% Yellow = 0;
% Red = 1;
% BallColour = Yellow;
%  
% i = 0;    % i represents z coordinate (i is 1.445 globally)
% m = 0;    % k is a condtional variable that triggers the bounce
% 
% % Creating an object to collide with
% centerpnt = [1.55,-0.2,1.145];
% centerpnt2 = [1.55,0.2,1.145];
% side = 0.2;
% plotOptions.plotFaces = true;
% [v,f,fn] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);              % Collide for Yellow
% [v2,f2,fn2] = RectangularPrism(centerpnt2-side/2, centerpnt2+side/2,plotOptions);         % Collide for Red
% 
% for j = 0:0.075:0.15            % j represents x cooridnate
%     tr = transl(j,0,i);
%     transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
%     transformedVertices2 = [vertices2,ones(size(vertices2,1),1)] * tr';
%     set(mesh_h,'vertices',transformedVertices(:,1:3));
%     set(mesh_h2,'vertices',transformedVertices2(:,1:3));
%     drawnow();
%     pause(0.25);
%     i = (i-0.02);           % No bouncing till later so no if 
% end
% 
% if (BallColour == 0)                                    % For yellow ball
%     RMRCMovement(Dobot,yellowCatchPoint,v,f,fn);
%     PlaceObject('CupNew.ply',[yellowCatchPoint(1)-0.075 yellowCatchPoint(2)-0.03 yellowCatchPoint(3)-0.125]);
% else                                                    % For red ball
%     RMRCMovement(Dobot,redCatchPoint,v2,f2,fn2);
%     PlaceObject('CupNew.ply',[redCatchPoint(1)+0.055 redCatchPoint(2)+0.03 redCatchPoint(3)-0.125]);
% end
% 
% for j = 0.15:0.075:1.45
%     tr = transl(j,0,i);
%     transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
%     transformedVertices2 = [vertices2,ones(size(vertices2,1),1)] * tr';
%     set(mesh_h,'vertices',transformedVertices(:,1:3));
%     set(mesh_h2,'vertices',transformedVertices2(:,1:3));
%     drawnow();
%     pause(0.25);
%     if (i <= -0.3)
%         m = 1;
%     end
%     if (m == 1)
%         i = (i+0.0075);
%     else
%         i = (i-0.02);
%     end
% end

%% GUI (Graphical User Interface) [Subject Resources]

% See the app designer file in the directory and adapt it

%% IBVS (Image Based Visual Servoing) [Lab 10]

% Definitions
% Create image target (points in the image plane) 
pStar = [662 362; 362 362];

%Create 3D points
P = [1.8,1.8; -0.25,0.25; 1.25,1.25];     

%Initial pose
q0 = [pi/2, -pi/6, 0 0 0];

% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, 'resolution', [1024 1024], 'centre', [512 512],'name', 'Dobot Camera');

% frame rate
fps = 25;

% Define values
% Gain of the controler
lambda = 0.6;

% Depth of the IBVS
depth = mean (P(1,:));

% Initialise Simulation (Display in 3D)
% Display Dobot
Tc0= Dobot.model.fkine(q0);
Dobot.model.animate(q0);
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
        J = cam.visjac_p(uv, depth);
    end

    % Compute the velocity of camera in camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        status = -1;
        return
    end
    fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);       % Print every column of the Jacobian (velocity matrix)

    % Compute robot's Jacobian and inverse
    J2 = Dobot.model.jacobn(q0);
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
    Dobot.model.animate(q);

    % Get camera location
    Tc = Dobot.model.fkine(Dobot.model.getpos());
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