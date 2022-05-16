
%% Pick and Place Code for Sensors and Control

close all;
clear all;
clc;

rosshutdown;        % Need to call this at the start
rosinit;            % Initialise connection

%% Initialise the Camera

sub = rossubscriber("camera/color/image_raw");
[msg2] = receive(sub);                                % Take the image
image = readImage(msg2);                              % Make the image readable so we can read
fprintf('\nCamera is initialised\n');

%% Find Centriods of Blocks

% Centres of coloured blocks
extractedRed = extractRed(image);                     % Outputs binary (0,1) image (RED pixels as 1, everything else as 0)
redCentre = calculateCentroid(extractedRed);          % Outputs pixel coordinates of red cube centre
figure, imshow(extractedRed);                         % Show the extracted RED in BW image (RED pixels are white, everything else is black)

extractedGreen = extractGreen(image);
greenCentre = calculateCentroid(extractedGreen);      % Outputs pixel coordinates of green cube centre
figure, imshow(extractedGreen);

extractedYellow = extractYellow(image);
yellowCentre = calculateCentroid(extractedYellow);    %  Outputs pixel coordinates of yellow cube centre
figure, imshow(extractedYellow);

extractedBlue = extractBlue(image);
blueCentre = calculateCentroid(extractedBlue);        %  Outputs pixel coordinates of blue cube centre
figure, imshow(extractedBlue);

extractedOrange = extractOrange(image);
orangeCentre = calculateCentroid(extractedOrange);    %  Outputs pixel coordinates of blue cube centre
figure, imshow(extractedOrange);

% Plot results
figure, imshow(image);
hold on

scatter(redCentre.Centroid(1), redCentre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(redCentre.Centroid(1) + 10, redCentre.Centroid(2),'RED','FontSize',14,'FontWeight','bold');              % +10 so the writing is legible

scatter(greenCentre.Centroid(1), greenCentre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(greenCentre.Centroid(1) + 10, greenCentre.Centroid(2),'GREEN','FontSize',14,'FontWeight','bold');

scatter(yellowCentre.Centroid(1), yellowCentre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(yellowCentre.Centroid(1) + 10, yellowCentre.Centroid(2),'YELLOW','FontSize',14,'FontWeight','bold');

scatter(blueCentre.Centroid(1), blueCentre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(blueCentre.Centroid(1) + 10, blueCentre.Centroid(2),'BLUE','FontSize',14,'FontWeight','bold');

scatter(orangeCentre.Centroid(1), orangeCentre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(orangeCentre.Centroid(1) + 10, orangeCentre.Centroid(2),'ORANGE','FontSize',14,'FontWeight','bold');

%% Calculate 3D Coordinates in Camera Frame

% Camera Parameters
focalLength = (917.6252+915.7077)/2;
PPx = 636.2969;
PPy = 351.4231;
 
% Depth (z-value) from Camera (conversion of mm to m)
zCameraToBlock = 400/1000;  

% Camera (xyz) coordinates
rCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,redCentre,zCameraToBlock);
gCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,greenCentre,zCameraToBlock);
yCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,yellowCentre,zCameraToBlock);
bCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,blueCentre,zCameraToBlock);
oCameraCoordinate = calculateCameraCoordinates(focalLength,PPx,PPy,orangeCentre,zCameraToBlock);

%% Convert Camera Coordinates to Dobot Coordinates
    
% Dobot Parameters
blockHeight = 0.03;
    
% Converting Camera Coordinates to Dobot Coordinates (each stored in a 1x3 matrix)
rDobot = [(0.2575+rCameraCoordinate(1,2)) rCameraCoordinate(1,1) -0.0375]              % x and y values are swapped from camera to Dobot wuth z values being hardcoded
rDobotHover = [(0.2575+rCameraCoordinate(1,2)) rCameraCoordinate(1,1) 0.1];            % 0.2575 is the x-value of the camera in the Dobot frame of reference

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
primaryDropoffHover = [0.2575, -0.072, 0.1];                  % Add in a hover step to ensure the Dobot doesn not knock over the blocks in translation
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

pause(25);                                      % Long pause as robot needs to be fully initialised before starting

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
     
send(targetEndEffectorPub,targetEndEffectorMsg);               % Send command to move

pause(3);

fprintf('\nMovement is primed\n');                        % Display to the user that the Dobot movement is primed

%% Priming Motions (Tool State)

state = 0;                        % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub,toolStateMsg);  % Send command to gripper

pause(2);

fprintf('\nSuction is primed\n');                        % Display to the user that the suction is primed

%% Movement of the Robot

% Moving Red Block
moveEndEffector(rDobotHover);
pause(3);
moveEndEffector(rDobot);
pause(2);
setToolState(1);
pause(2);
moveEndEffector(rDobotHover);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(3);
moveEndEffector(primaryDropoff1);
pause(2);
setToolState(0);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(2);

% Moving Yellow Block
moveEndEffector(yDobotHover);
pause(3);
moveEndEffector(yDobot);
pause(2);
setToolState(1);
pause(2);
moveEndEffector(yDobotHover);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(3);
moveEndEffector(primaryDropoff2);
pause(2);
setToolState(0);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(2);

% Moving Blue Block
moveEndEffector(bDobotHover);
pause(3);
moveEndEffector(bDobot);
pause(2);
setToolState(1);
pause(2);
moveEndEffector(bDobotHover);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(3);
moveEndEffector(primaryDropoff2);
pause(2);
setToolState(0);
pause(2);
moveEndEffector(primaryDropoffHover);
pause(2);

% Moving Green Block
moveEndEffector(gDobotHover);
pause(3);
moveEndEffector(gDobot);
pause(2);
setToolState(1);
pause(2);
moveEndEffector(gDobotHover);
pause(2);
moveEndEffector(secondaryDropoffHover);
pause(3);
moveEndEffector(secondaryDropoff1);
pause(2);
setToolState(0);
pause(2);
moveEndEffector(secondaryDropoffHover);
pause(2);

% Moving Orange Block
moveEndEffector(oDobotHover);
pause(3);
moveEndEffector(oDobot);
pause(2);
setToolState(1);
pause(2);
moveEndEffector(oDobotHover);
pause(2);
moveEndEffector(secondaryDropoffHover);
pause(3);
moveEndEffector(secondaryDropoff2);
pause(2);
setToolState(0);
pause(2);
moveEndEffector(secondaryDropoffHover);
