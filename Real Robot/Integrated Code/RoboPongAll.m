
%% Pick & Place Code for Sensors & Control
 
% close all; 
% clear all; 
% clc;
% 
% rosshutdown;        % Need to call this at the start 
% rosinit;            % Initialise connection

%% Initialise the Camera

sub = rossubscriber("camera/color/image_raw");
[msg2] = receive(sub);
image = readImage(msg2);
fprintf('\nCamera is initialised\n');

%% Find Centriods of Blocks

extracted_orange = extract_orange2(image);
orange_centre = calculateCentroid(extracted_orange);    %  Outputs pixel coordinates of blue cube centre
figure, imshow(extracted_orange);

% Plot results
figure, imshow(image);
hold on

scatter(orange_centre.Centroid(1), orange_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(orange_centre.Centroid(1) + 10, orange_centre.Centroid(2),'ORANGE','FontSize',14,'FontWeight','bold');

%% Calculate 3D Coordinates in Camera Frame

% Camera Parameters
focalLength = (917.6252+915.7077)/2;
PPx = 636.2969;
PPy = 351.4231;
 
% Depth (z value) from Camera (conversion of mm to m)
zCameraToBlock = 992/1000;  

% Camera (xyz) coordinates
oCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,orange_centre,zCameraToBlock);

%% Convert Camera Coordinates to Dobot Coordinates
    
% Converting Camera Coordinates to Dobot Coordinates (each stored in a 1x3 matrix)
oDobot = [0.951 oCameraCoordinate(1,1) 0.59-oCameraCoordinate(1,2)]

%% Initalise Dobot
  
% [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
% safetyStateMsg.Data = 2;
% send(safetyStatePublisher,safetyStateMsg);
% 
% pause(25);          % Long pause as robot needs to be fully initialised before starting
% 
% fprintf('\nDobot is initialised\n');

%% Priming Motions (Movement)

target = [0.2575,0,0.03];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    
targetEndEffectorMsg.Position.X = target(1);
targetEndEffectorMsg.Position.Y = target(2);
targetEndEffectorMsg.Position.Z = target(3);
    
target_rotation = [0,0,0];
qua = eul2quat(target_rotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);
     
send(targetEndEffectorPub,targetEndEffectorMsg);         % Send command to move

pause(3)

fprintf('\nMovement is primed\n');            % Display end effector current position

%% Priming Motions (Tool State)

state = 1;                        % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub,toolStateMsg);  % Send command to gripper

pause (2)

fprintf('\nSuction is primed\n');            % Display message for user

%% Movement Left

DropoffLeft = [0, -0.275, 0.03];
target = DropoffLeft;

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    
targetEndEffectorMsg.Position.X = target(1);
targetEndEffectorMsg.Position.Y = target(2);
targetEndEffectorMsg.Position.Z = target(3);
    
target_rotation = [0,0,0];
qua = eul2quat(target_rotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);
     
send(targetEndEffectorPub,targetEndEffectorMsg);         % Send command to move

pause(3)

fprintf('\nDobot has moved left\n');            % Display end effector current position

%% CLear Workspace & Command

pause(3);
clear all;
clc;

%% Initialise the Camera

sub = rossubscriber("camera/color/image_raw");
[msg2] = receive(sub);
image = readImage(msg2);
fprintf('\nCamera is initialised\n');

%% Find Centriods of Blocks

extracted_orange = extract_orange2(image);
orange_centre = calculateCentroid(extracted_orange);    %  Outputs pixel coordinates of blue cube centre
figure, imshow(extracted_orange);

% Plot results
figure, imshow(image);
hold on

scatter(orange_centre.Centroid(1), orange_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(orange_centre.Centroid(1) + 10, orange_centre.Centroid(2),'ORANGE','FontSize',14,'FontWeight','bold');

%% Calculate 3D Coordinates in Camera Frame

% Camera Parameters
focalLength = (917.6252+915.7077)/2;
PPx = 636.2969;
PPy = 351.4231;
 
% Depth (z value) from Camera (conversion of mm to m)
zCameraToBlock = 992/1000;  

% Camera (xyz) coordinates
oCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,orange_centre,zCameraToBlock);

%% Convert Camera Coordinates to Dobot Coordinates
    
% Converting Camera Coordinates to Dobot Coordinates (each stored in a 1x3 matrix)
oDobot = [0.951 oCameraCoordinate(1,1) 0.59-oCameraCoordinate(1,2)]

%% Movement Left Low

DropoffLeft = [0, -0.275, -0.03];
target = DropoffLeft;

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    
targetEndEffectorMsg.Position.X = target(1);
targetEndEffectorMsg.Position.Y = target(2);
targetEndEffectorMsg.Position.Z = target(3);
    
target_rotation = [0,0,0];
qua = eul2quat(target_rotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);
     
send(targetEndEffectorPub,targetEndEffectorMsg);         % Send command to move

pause(3)

fprintf('\nDobot has moved left\n');            % Display end effector current position

%% CLear Workspace & Command

pause(3);
clear all;
clc;

%% Initialise the Camera

sub = rossubscriber("camera/color/image_raw");
[msg2] = receive(sub);
image = readImage(msg2);
fprintf('\nCamera is initialised\n');

%% Find Centriods of Blocks

extracted_orange = extract_orange2(image);
orange_centre = calculateCentroid(extracted_orange);    %  Outputs pixel coordinates of blue cube centre
figure, imshow(extracted_orange);

% Plot results
figure, imshow(image);
hold on

scatter(orange_centre.Centroid(1), orange_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(orange_centre.Centroid(1) + 10, orange_centre.Centroid(2),'ORANGE','FontSize',14,'FontWeight','bold');

%% Calculate 3D Coordinates in Camera Frame

% Camera Parameters
focalLength = (917.6252+915.7077)/2;
PPx = 636.2969;
PPy = 351.4231;
 
% Depth (z value) from Camera (conversion of mm to m)
zCameraToBlock = 992/1000;  

% Camera (xyz) coordinates
oCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,orange_centre,zCameraToBlock);

%% Convert Camera Coordinates to Dobot Coordinates
    
% Converting Camera Coordinates to Dobot Coordinates (each stored in a 1x3 matrix)
oDobot = [0.951 oCameraCoordinate(1,1) 0.59-oCameraCoordinate(1,2)]

%% Movement Left High

DropoffLeft = [0, -0.275, 0.09];
target = DropoffLeft;

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    
targetEndEffectorMsg.Position.X = target(1);
targetEndEffectorMsg.Position.Y = target(2);
targetEndEffectorMsg.Position.Z = target(3);
    
target_rotation = [0,0,0];
qua = eul2quat(target_rotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);
     
send(targetEndEffectorPub,targetEndEffectorMsg);         % Send command to move

pause(3)

fprintf('\nDobot has moved left\n');            % Display end effector current position