
%% Main code for Sensors & Control [Pick & Place]

close all;
clear all;
clc;

rosshutdown;        % Call this at the start just in case
rosinit;            % Initialise connection

%% Initialise Dobot

initialiseDobot();
 
default_pos = [0.2591,0,-0.0086];
ground_level = -0.0419;

%% Get Current Safety Status

SafetyStatus = getSafetyStatus();

%% Return End Effector Position

EndEffectorPosition = getEndEffectorPosition();

%% Movement of the End Effector

target = [0.2591,0,0.3];
moveEndEffector(target);

%% Toggle Gripper State

ToolState = getToolState();

gripped = 1;
released = 0;
state = gripped;                        % Change to grip or release

setToolState(state);
send(toolStatePub,toolStateMsg);        % Send command to grip

ToolStateToggled = getToolState();

%% End Program

rosshutdown;
