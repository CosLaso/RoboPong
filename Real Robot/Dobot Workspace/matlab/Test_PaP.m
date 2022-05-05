
%% Main code for Sensors & Control [Pick & Place]

% Any command with send in it has issues working from within a function

close all;
clear all;
clc;

rosshutdown;        % Call this at the start just in case
rosinit;            % Initialise connection

%% Initialise Dobot

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);
fprintf('Dobot is initialised with the current parameters\n');
 
defaultEndEffectorPosition = [0.2591,0,-0.0086]    % Default end effector position
groundLevel = -0.0419                              % Z value of the table
safetyStatus = getSafetyStatus();                  % Return the safety status of the robot

%% Movement of the End Effector

fprintf('Dobot is moving to');              % Display end effector target position
target = [0.2591,0,0.5]

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
     
send(targetEndEffectorPub,targetEndEffectorMsg);       % Send command to move
fprintf('Dobot has completed translation/n');            % Display end effector current position

%% Toggle Gripper State

getToolState();                   % Display tool state
state = 0;                        % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub,toolStateMsg);  % Send command to gripper

getToolState();                   % Display tool state

%% End Program

rosshutdown;