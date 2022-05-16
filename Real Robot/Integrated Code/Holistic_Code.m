
%% Pick & Place Code for Sensors & Control

close all;
clear all;
clc;

rosshutdown;        % Need to call this at the start
rosinit;            % Initialise connection

%% Initialise the Camera

sub = rossubscriber("camera/color/image_raw");
[msg2] = receive(sub);
image = readImage(msg2);
fprintf('\nCamera is initialised\n');

%% Find Centriods of Blocks

% Centres of coloured blocks
extracted_red = extract_red(image);                     % Outputs binary (0,1) same size array with red as 1
red_centre = calculateCentroid(extracted_red);          % Outputs pixel coordinates of red cube centre
% figure, imshow(extracted_red);                          % Show the extracted RED (RED pixels are white, every other pixel is black)

extracted_green = extract_green(image);
green_centre = calculateCentroid(extracted_green);      % Outputs pixel coordinates of green cube centre
% figure, imshow(extracted_green);

extracted_yellow = extract_yellow(image);
yellow_centre = calculateCentroid(extracted_yellow);    %  Outputs pixel coordinates of yellow cube centre
% figure, imshow(extracted_yellow);

extracted_blue = extract_blue(image);
blue_centre = calculateCentroid(extracted_blue);        %  Outputs pixel coordinates of blue cube centre
% figure, imshow(extracted_blue);

extracted_orange = extract_orange(image);
orange_centre = calculateCentroid(extracted_orange);    %  Outputs pixel coordinates of blue cube centre
% figure, imshow(extracted_orange);

% Plot results
figure, imshow(image);
hold on

scatter(red_centre.Centroid(1), red_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(red_centre.Centroid(1) + 10, red_centre.Centroid(2),'RED','FontSize',14,'FontWeight','bold');

scatter(green_centre.Centroid(1), green_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(green_centre.Centroid(1) + 10, green_centre.Centroid(2),'GREEN','FontSize',14,'FontWeight','bold');

scatter(yellow_centre.Centroid(1), yellow_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(yellow_centre.Centroid(1) + 10, yellow_centre.Centroid(2),'YELLOW','FontSize',14,'FontWeight','bold');

scatter(blue_centre.Centroid(1), blue_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(blue_centre.Centroid(1) + 10, blue_centre.Centroid(2),'BLUE','FontSize',14,'FontWeight','bold');

scatter(orange_centre.Centroid(1), orange_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(orange_centre.Centroid(1) + 10, orange_centre.Centroid(2),'ORANGE','FontSize',14,'FontWeight','bold');

%% Calculate 3D Coordinates in Camera Frame

% Camera Parameters
focalLength = (917.6252+915.7077)/2;
PPx = 636.2969;
PPy = 351.4231;
 
% Depth (z value) from Camera (conversion of mm to m)
zCameraToBlock = 400/1000;  

% Camera (xyz) coordinates
rCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,red_centre,zCameraToBlock);
gCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,green_centre,zCameraToBlock);
yCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,yellow_centre,zCameraToBlock);
bCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,blue_centre,zCameraToBlock);
oCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,orange_centre,zCameraToBlock);

%% Convert Camera Coordinates to Dobot Coordinates
    
% Dobot Parameters
% groundToDobotCentre = 0.057;
blockHeight = 0.03;
% tableheight = 0.0900;
    
% Converting Camera Coordinates to Dobot Coordinates (each stored in a 1x3 matrix)
rDobot = [(0.2575+rCameraCoordinate(1,2)) rCameraCoordinate(1,1) -0.0375]        
rDobotHover = [(0.2575+rCameraCoordinate(1,2)) rCameraCoordinate(1,1) 0.1];

gDobot = [(0.2575+gCameraCoordinate(1,2)) gCameraCoordinate(1,1) -0.0375]
gDobotHover = [(0.2575+gCameraCoordinate(1,2)) gCameraCoordinate(1,1) 0.1];

yDobot = [(0.2575+yCameraCoordinate(1,2)) yCameraCoordinate(1,1) -0.0375]
yDobotHover = [(0.2575+yCameraCoordinate(1,2)) yCameraCoordinate(1,1) 0.1];

bDobot = [(0.2575+bCameraCoordinate(1,2)) bCameraCoordinate(1,1) -0.0375]
bDobotHover = [(0.2575+bCameraCoordinate(1,2)) bCameraCoordinate(1,1) 0.1];

oDobot = [(0.2575+oCameraCoordinate(1,2)) oCameraCoordinate(1,1) -0.0375]
oDobotHover = [(0.2575+oCameraCoordinate(1,2)) oCameraCoordinate(1,1) 0.1];
    
%% Dropoff Coordinates (Hardcoded Values)

% For primary colours (Red,Yellow,Blue)
primaryDropoffHover = [0.2575, -0.072, 0.1];
primaryDropoff1 = [0.2575, -0.075, -0.03];
primaryDropoff2 = [0.2575, -0.075, -0.03+blockHeight];
primaryDropoff3 = [0.2575, -0.075, -0.03+(2*blockHeight)];
 
% For secondary colours (Orange,Purple)
secondaryDropoffHover = [0.2575, 0.075, 0.1];
secondaryDropoff1 = [0.2575, 0.075, -0.03];
secondaryDropoff2 = [0.2575, 0.075, -0.03+blockHeight];
    
%% Initalise Dobot
  
[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

pause(25);          % Long pause as robot needs to be fully initialised before starting

fprintf('\nDobot is initialised\n');

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

state = 0;                        % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub,toolStateMsg);  % Send command to gripper

pause (2)

fprintf('\nSuction is primed\n');            % Display message for user

%% Movement of the Robot

% Moving Red Block
moveEndEffector(rDobotHover);
pause(5);
moveEndEffector(rDobot);
pause(3);
setToolState(1);
pause(2);
moveEndEffector(rDobotHover);
pause(3);
moveEndEffector(primaryDropoffHover);
pause(3);
moveEndEffector(primaryDropoff1);
pause(3);
setToolState(0);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(3);

% Moving Yellow Block
moveEndEffector(yDobotHover);
pause(5);
moveEndEffector(yDobot);
pause(3);
setToolState(1);
pause(2);
moveEndEffector(yDobotHover);
pause(3);
moveEndEffector(primaryDropoffHover);
pause(5);
moveEndEffector(primaryDropoff2);
pause(3);
setToolState(0);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(3);

% Moving Blue Block
moveEndEffector(bDobotHover);
pause(5);
moveEndEffector(bDobot);
pause(3);
setToolState(1);
pause(2);
moveEndEffector(bDobotHover);
pause(3);
moveEndEffector(primaryDropoffHover);
pause(5);
moveEndEffector(primaryDropoff2);
pause(3);
setToolState(0);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(3);

% Moving Green Block
moveEndEffector(gDobotHover);
pause(5);
moveEndEffector(gDobot);
pause(3);
setToolState(1);
pause(2);
moveEndEffector(gDobotHover);
pause(3);
moveEndEffector(secondaryDropoffHover);
pause(5);
moveEndEffector(secondaryDropoff1);
pause(3);
setToolState(0);
pause(2);
moveEndEffector(secondaryDropoffHover);
pause(3);

% Moving Orange Block
moveEndEffector(oDobotHover);
pause(5);
moveEndEffector(oDobot);
pause(3);
setToolState(1);
pause(2);
moveEndEffector(oDobotHover);
pause(3);
moveEndEffector(secondaryDropoffHover);
pause(5);
moveEndEffector(secondaryDropoff2);
pause(3);
setToolState(0);
pause(2);
moveEndEffector(secondaryDropoffHover);
