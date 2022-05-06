%% Intialise Connection

rosinit;

%% Robot Safety Status

safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); %Allow some time for MATLAB to start the subscriber
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;
fprintf('Safety Status: %d\n',currentSafetyStatus);

%% Intialise Dobot

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

default_pos = [0.2589,0,-0.0085];
ground_level = -0.0589;

%% Set Software E-Stop

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 3;
send(safetyStatePublisher,safetyStateMsg);

%% Movement

%Move down, pick up object and move back up
Move_End_Effector(next_target);
tool_state(1);
pause(3);
Move_End_Effector(default_pos);

pause(5)

%Move down, place object and move back up
Move_End_Effector(next_target);
pause(3);
tool_state(0);
Move_End_Effector(default_pos);

%% End Program

rosshutdown;
