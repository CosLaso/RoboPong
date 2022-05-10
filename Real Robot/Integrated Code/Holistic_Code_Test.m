
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
figure, imshow(extracted_red);                          % Show the extracted RED (RED pixels are white, every other pixel is black)

extracted_green = extract_green(image);
green_centre = calculateCentroid(extracted_green);      % Outputs pixel coordinates of green cube centre
figure, imshow(extracted_green);

extracted_yellow = extract_yellow(image);
yellow_centre = calculateCentroid(extracted_yellow);    %  Outputs pixel coordinates of yellow cube centre
figure, imshow(extracted_yellow);

extracted_blue = extract_blue(image);
blue_centre = calculateCentroid(extracted_blue);        %  Outputs pixel coordinates of blue cube centre
figure, imshow(extracted_blue);

extracted_orange = extract_orange(image);
orange_centre = calculateCentroid(extracted_orange);    %  Outputs pixel coordinates of blue cube centre
figure, imshow(extracted_orange);

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
zCameraToBlock = 557/1000;  

% Camera (xyz) coordinates
rCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,red_centre,zCameraToBlock);
gCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,green_centre,zCameraToBlock);
yCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,yellow_centre,zCameraToBlock);
bCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,blue_centre,zCameraToBlock);
oCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,orange_centre,zCameraToBlock);

%% Convert Camera Coordinates to Dobot Coordinates
    
% Dobot Parameters
groundToDobotCentre = 0.057;
blockHeight = 0.0260;
tableheight = 0.0900;
    
% Converting Camera Coordinates to Dobot Coordinates (each stored in a 1x3 matrix)
rDobot = [0.26925-rCameraCoordinate(1,2) (-1)*rCameraCoordinate(1,1) 0.08]         
rDobotHover = [0.26925-rCameraCoordinate(1,2) (-1)*rCameraCoordinate(1,1) 0-groundToDobotCentre+blockHeight];

gDobot = [0.26925-gCameraCoordinate(1,2) (-1)*gCameraCoordinate(1,1) 0.08]
gDobotHover = [0.26925-gCameraCoordinate(1,2) (-1)*gCameraCoordinate(1,1) 0-groundToDobotCentre+blockHeight];

yDobot = [0.26925-yCameraCoordinate(1,2) (-1)*yCameraCoordinate(1,1) 0.08]
yDobotHover = [0.26925-yCameraCoordinate(1,2) (-1)*yCameraCoordinate(1,1) 0-groundToDobotCentre+blockHeight];

bDobot = [0.26925-bCameraCoordinate(1,2) (-1)*bCameraCoordinate(1,1) 0.08]
bDobotHover = [0.26925-bCameraCoordinate(1,2) (-1)*bCameraCoordinate(1,1) 0-groundToDobotCentre+blockHeight];

oDobot = [0.26925-oCameraCoordinate(1,2) (-1)*oCameraCoordinate(1,1) 0.08]
oDobotHover = [0.26925-oCameraCoordinate(1,2) (-1)*oCameraCoordinate(1,1) 0-groundToDobotCentre+blockHeight];
    
%% Dropoff Coordinates (Hardcoded Values)

% For primary colours (Red,Yellow,Blue)
primaryDropoffHover = [0.26925, -0.072, 0.1];
primaryDropoff1 = [0.26925, -0.072, -0.03];
primaryDropoff2 = [0.26925, -0.072, -0.03+blockHeight];
primaryDropoff3 = [0.26925, -0.072, -0.03++(2*blockHeight)];
 
% For secondary colours (Orange,Purple)
secondaryDropoffHover = [0.26925, 0.072, 0.1];
secondaryDropoff1 = [0.26925, 0.072, -0.03];
secondaryDropoff2 = [0.26925, 0.072, -0.03+blockHeight];
    
%% Initalise Dobot
  
[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

pause(25);          % Long pause as robot needs to be fully initialised before starting

fprintf('\nDobot is initialised\n');

%% Movement of the Robot

% Moving Red Block
moveEndEffector(rDobotHover);
pause(5);
moveEndEffector(rDobot);
pause(2);
setToolState(1);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(5);
moveEndEffector(primaryDropoff1);
pause(2);
setToolState(0);
pause(2);

% Moving Yellow Block
moveEndEffector(yDobotHover);
pause(5);
moveEndEffector(yDobot);
pause(2);
setToolState(1);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(5);
moveEndEffector(primaryDropoff2);
pause(2);
setToolState(0);
pause(2);

% Moving Blue Block
moveEndEffector(bDobotHover);
pause(5);
moveEndEffector(bDobot);
pause(2);
setToolState(1);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(5);
moveEndEffector(primaryDropoff2);
pause(2);
setToolState(0);
pause(2);

% Moving Green Block
moveEndEffector(gDobotHover);
pause(5);
moveEndEffector(gDobot);
pause(2);
setToolState(1);
pause(2);
moveEndEffector(secondaryaryDropoffHover);
pause(5);
moveEndEffector(secondaryDropoff1);
pause(2);
setToolState(0);
pause(2);

% Moving Orange Block
moveEndEffector(oDobotHover);
pause(5);
moveEndEffector(oDobot);
pause(2);
setToolState(1);
pause(2);
moveEndEffector(secondaryaryDropoffHover);
pause(5);
moveEndEffector(secondaryDropoff2);
pause(2);
setToolState(0);
