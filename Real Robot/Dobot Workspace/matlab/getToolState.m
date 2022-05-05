function [currentToolState] = getToolState()

% For Gripper 0 -> Gripped (OFF) | 1 -> Released (ON)
    toolStateSubscriber = rossubscriber('/dobot_magician/tool_state');
    pause(2); % Allow some time for MATLAB to start the subscriber
    currentToolState = toolStateSubscriber.LatestMessage.Data;
    fprintf('Tool State: %d\n',currentToolState);
    
end

