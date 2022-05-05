
close all;
clear all;
clc;
rosshutdown;

%% Initialse Connection

rosinit

%% Get Current Safety Status

safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); % Allow some time for MATLAB to start the subscriber
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;

%% Initialise Dobot 

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

%% Get Current Joint State

jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
pause(2); % Allow some time for a message to appear
currentJointState = jointStateSubscriber.LatestMessage.Position % Get the latest message

%% Get Current End Effector Pose

endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses'); % Create a ROS Subscriber to the topic end_effector_poses
pause(2); %Allow some time ();for MATLAB to start the subscriber
currentEndEffectorPoseMsg = endEffectorPoseSubscriber.LatestMessage;

% Extract the position of the end effector from the received message
currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X,
                              currentEndEffectorPoseMsg.Pose.Position.Y,
                              currentEndEffectorPoseMsg.Pose.Position.Z]

% Extract the orientation of the end effector
currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W,
                          currentEndEffectorPoseMsg.Pose.Orientation.X,
                          currentEndEffectorPoseMsg.Pose.Orientation.Y,
                          currentEndEffectorPoseMsg.Pose.Orientation.Z];

% Convert from quaternion to euler
% [roll,pitch,yaw] = quat2eul(currentEndEffectorQuat);

%% Set Target Joint State

jointTarget = [0,0.4,0.3,0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

%% Set Target End Effector State

endEffectorPosition = [0.2,0,0.1];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);

%% Get Current Tool State

toolStateSubscriber = rossubscriber('/dobot_magician/tool_state');
pause(2); %Allow some time for MATLAB to start the subscriber
currentToolState = toolStateSubscriber.LatestMessage.Data;

%% Set Tool State (Set Gripper)

% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 0];
send(toolStatePub,toolStateMsg);

%% Get Current IO of Data of the Robot

ioDataSubscriber = rossubscriber('/dobot_magician/io_data');
pause(2); %Allow some time for MATLAB to start the subscriber
ioMultiplex = ioDataSubscriber.LatestMessage.Data(2:21);
ioData = ioDataSubscriber.LatestMessage.Data(23:42);

%% Set IO Port Data

address = 1;
ioMux = 1;
data = 1;
[ioDataPub,ioDataMsg] = rospublisher('/dobot_magician/target_io_state');
ioData = [address,ioMux,data];
ioDataMsg.Data = ioData;
send(ioDataPub,ioDataMsg);
