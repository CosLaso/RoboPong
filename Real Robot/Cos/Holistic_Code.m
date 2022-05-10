
%% Pick & Place Code for Sensors & Control

close all;
clear all;
clc;

rosshutdown;        % Call this at the start just in case
rosinit;            % Initialise connection

%% Initialise the Camera

sub = rossubscriber("camera/color/image_raw");
[msg2] = receive(sub);
image = readImage(msg2);
fprintf('Camera is initialised\n');

%% Find Centriods of Blocks
    
% Centre of Red
extracted_red = extract_red(image);                     % Outputs binary (0,1) same size array with red as 1
red_centre = calculate_centroid(extracted_red);         % Outputs pixel coordinates of red cube centre
% figure, imshow(extracted_red);                          % Show the extracted RED (RED pixels are white, every other pixel is black)

% Centre of Green
extracted_green = extract_green(image);
green_centre = calculate_centroid(extracted_green);     % Outputs pixel coordinates of green cube centre
% figure, imshow(extracted_green);

% Centre of Yellow
extracted_yellow = extract_yellow(image);
yellow_centre = calculate_centroid(extracted_yellow);   %  Outputs pixel coordinates of yellow cube centre
% figure, imshow(extracted_yellow);

% Centre of Blue
extracted_blue = extract_blue(image);
blue_centre = calculate_centroid(extracted_blue);   %  Outputs pixel coordinates of blue cube centre
% figure, imshow(extracted_blue);

% Centre of Orange
extracted_orange = extract_orange(image);
orange_centre = calculate_centroid(extracted_orange);   %  Outputs pixel coordinates of blue cube centre
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

% Move Origin to Principle Point:
Red_X_L = red_centre.Centroid(1,1) - PPx;
Red_Y_L = red_centre.Centroid(1,2) - PPy;

Green_X_L = green_centre.Centroid(1,1) - PPx;
Green_Y_L = green_centre.Centroid(1,2) - PPy;

Yellow_X_L = yellow_centre.Centroid(1,1) - PPx;
Yellow_Y_L = yellow_centre.Centroid(1,2) - PPy;

Blue_X_L = blue_centre.Centroid(1,1) - PPx;
Blue_Y_L = blue_centre.Centroid(1,2) - PPy;

Orange_X_L = orange_centre.Centroid(1,1) - PPx;
Orange_Y_L = orange_centre.Centroid(1,2) - PPy;
        
% Collect Depth (Z value) information from Camera (conversion of mm to m)
Z_Cam_To_Block = 557/1000;        

% Calculate X and Y Coordinates using already known Z
Red_X_Camera = (Z_Cam_To_Block*Red_X_L) / focalLength;
Red_Y_Camera = (Z_Cam_To_Block*Red_Y_L) / focalLength;

Green_X_Camera = (Z_Cam_To_Block*Green_X_L) / focalLength;
Green_Y_Camera = (Z_Cam_To_Block*Green_Y_L) / focalLength;

Yellow_X_Camera = (Z_Cam_To_Block*Yellow_X_L) / focalLength;
Yellow_Y_Camera = (Z_Cam_To_Block*Yellow_Y_L) / focalLength;

Blue_X_Camera = (Z_Cam_To_Block*Blue_X_L) / focalLength;
Blue_Y_Camera = (Z_Cam_To_Block*Blue_Y_L) / focalLength;

Orange_X_Camera = (Z_Cam_To_Block*Orange_X_L) / focalLength;
Orange_Y_Camera = (Z_Cam_To_Block*Orange_Y_L) / focalLength;

% Store in 1 by 3 Matrix of Coordinates (x,y,z)
RED_CamCoordinate = [Red_X_Camera Red_Y_Camera Z_Cam_To_Block];
GREEN_CamCoordinate = [Green_X_Camera Green_Y_Camera Z_Cam_To_Block];
YELLOW_CamCoordinate = [Yellow_X_Camera Yellow_Y_Camera Z_Cam_To_Block];
BLUE_CamCoordinate = [Blue_X_Camera Blue_Y_Camera Z_Cam_To_Block];
ORANGE_CamCoordinate = [Orange_X_Camera Orange_Y_Camera Z_Cam_To_Block];

%% Convert Camera Coordinates to Dobot Coordinates
    
% Dobot Parameters
groundToDobotCentre = 0.057;
blockHeight = 0.0260;
tableheight = 0.0900;
    
% Converting Camera Coordinates to Dobot Coordinates (1x3 Matrix of Coordinates each)
rDobot = [0.26925-RED_CamCoordinate(1,2) (-1)*RED_CamCoordinate(1,1) 0.08]         
rDobotHover = [0.26925-RED_CamCoordinate(1,2) (-1)*RED_CamCoordinate(1,1) 0-groundToDobotCentre+blockHeight];

gDobot = [0.26925-GREEN_CamCoordinate(1,2) (-1)*GREEN_CamCoordinate(1,1) 0.08]
gDobotHover = [0.26925-GREEN_CamCoordinate(1,2) (-1)*GREEN_CamCoordinate(1,1) 0-groundToDobotCentre+blockHeight];

yDobot = [0.26925-YELLOW_CamCoordinate(1,2) (-1)*YELLOW_CamCoordinate(1,1) 0.08]
yDobotHover = [0.26925-BLUE_CamCoordinate(1,2) (-1)*BLUE_CamCoordinate(1,1) 0-groundToDobotCentre+blockHeight];

bDobot = [0.26925-BLUE_CamCoordinate(1,2) (-1)*BLUE_CamCoordinate(1,1) 0.08]
bDobotHover = [0.26925-BLUE_CamCoordinate(1,2) (-1)*BLUE_CamCoordinate(1,1) 0-groundToDobotCentre+blockHeight];

oDobot = [0.26925-ORANGE_CamCoordinate(1,2) (-1)*ORANGE_CamCoordinate(1,1) 0.08]
oDobotHover = [0.26925-ORANGE_CamCoordinate(1,2) (-1)*ORANGE_CamCoordinate(1,1) 0-groundToDobotCentre+blockHeight];
    
% Dropoff Coordinates    
primaryDropoffHover = [0.26925, -0.072, 0.1];
primaryDropoff1 = [0.26925, -0.072, -0.03];
primaryDropoff2 = [0.26925, -0.072, -0.03+blockHeight];
primaryDropoff3 = [0.26925, -0.072, -0.03++(2*blockHeight)];
 
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
