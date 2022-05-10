function [] = setToolState(state)

% 0 -> OFF | 1 -> ON

    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    toolStateMsg.Data = [state];
    send(toolStatePub,toolStateMsg);

end

