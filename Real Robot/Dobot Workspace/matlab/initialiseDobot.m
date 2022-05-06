function [] = initialiseDobot()

% Initialise Dobot
    [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 2;
    send(safetyStatePublisher,safetyStateMsg);
    fprintf('Dobot is initialised\n');
    
end

