%% Program Settings

valid_method = 0;

while valid_method == 0
    fprintf('Movement Modes\n1) No Movement\n2) Joint State\n3) End Effector Pose\n4) Input Movements\n');
    movement_method = input("Select Movement Mode: ");
    movement_method = round(movement_method);
    if movement_method <= 4 && movement_method >= 1
        valid_method = 1;
    else
        fprintf('\nNot a valid mode, please try again\n\n');
    end
end

%% Intialise Connection

rosinit;

%% Robot Safety Status

safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); %Allow some time for MATLAB to start the subscriber
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;

%% Intialise Dobot

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

%% Set Software E-Stop

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 3;
send(safetyStatePublisher,safetyStateMsg);

%% Pose Analysis

if movement_method == 1
    %Get current joint state
    jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
    pause(2); % Allow some time for a message to appear
    currentJointState = jointStateSubscriber.LatestMessage.Position % Get the latest message
    
    %Get current end effector pose
    endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses'); % Create a ROS Subscriber to the topic end_effector_poses
    pause(2); %Allow some time for MATLAB to start the subscriber
    currentEndEffectorPoseMsg = endEffectorPoseSubscriber.LatestMessage;
    % Extract the position of the end effector from the received message
    currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X,
                                  currentEndEffectorPoseMsg.Pose.Position.Y,
                                  currentEndEffectorPoseMsg.Pose.Position.Z];
    % Extract the orientation of the end effector
    currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W,
                              currentEndEffectorPoseMsg.Pose.Orientation.X,
                              currentEndEffectorPoseMsg.Pose.Orientation.Y,
                              currentEndEffectorPoseMsg.Pose.Orientation.Z];
    % Convert from quaternion to euler
    [roll,pitch,yaw] = quat2eul(currentEndEffectorQuat);
end

%% Joint States

if movement_method == 2
    %Get current joint state
    jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
    pause(2); % Allow some time for a message to appear
    currentJointState = jointStateSubscriber.LatestMessage.Position % Get the latest message

    %Set joint state target
    jointTarget = [-0.5,0.2,0.2,0]; % Remember that the Dobot has 4 joints by default.

    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;

    send(targetJointTrajPub,targetJointTrajMsg);
end

%% End Effector State

if movement_method == 3
    %Get current end effector pose
    endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses'); % Create a ROS Subscriber to the topic end_effector_poses
    pause(2); %Allow some time for MATLAB to start the subscriber
    currentEndEffectorPoseMsg = endEffectorPoseSubscriber.LatestMessage;
    % Extract the position of the end effector from the received message
    currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X,
                                  currentEndEffectorPoseMsg.Pose.Position.Y,
                                  currentEndEffectorPoseMsg.Pose.Position.Z];
    % Extract the orientation of the end effector
    currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W,
                              currentEndEffectorPoseMsg.Pose.Orientation.X,
                              currentEndEffectorPoseMsg.Pose.Orientation.Y,
                              currentEndEffectorPoseMsg.Pose.Orientation.Z];
    % Convert from quaternion to euler
    %[roll,pitch,yaw] = quat2eul(currentEndEffectorQuat);

    %Set end effector pose target
    endEffectorPosition = [0.1,0,0.1];
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
    %%
    while(1)
        %Get current end effector pose
        endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses'); % Create a ROS Subscriber to the topic end_effector_poses
        pause(2); %Allow some time for MATLAB to start the subscriber
        currentEndEffectorPoseMsg = endEffectorPoseSubscriber.LatestMessage;
        % Extract the position of the end effector from the received message
        currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X,
                                      currentEndEffectorPoseMsg.Pose.Position.Y,
                                      currentEndEffectorPoseMsg.Pose.Position.Z];                   
        %Print positions
        fprintf('X: %d\nY: %d\nZ: %d\n\n',currentEndEffectorPosition(1),currentEndEffectorPosition(2),currentEndEffectorPosition(3));
        pause(2);
    end

end

%% Input Movements

if movement_method == 4
    while(1)
        %User input
        valid_input = 0;
        x = input("X coordinate: ");
        y = input("Y coordinate: ");
        z = input("Z coordinate: ");
        
        while valid_input == 0
            tool_state = input("\nTool State\n1 - ON\n0 - OFF\nSelection: ");

            if tool_state <= 2 && movement_method >= 1
                valid_input = 1;
            else
                fprintf('\nNot a valid state, please try again\n\n');
            end
        end

        %Set end effector pose target
        endEffectorPosition = [x,y,z];
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
        
        %Set tool state
        [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
        toolStateMsg.Data = [tool_state]; % Send 1 for on and 0 for off 
        send(toolStatePub,toolStateMsg);
    end
end

%% End Program

rosshutdown;