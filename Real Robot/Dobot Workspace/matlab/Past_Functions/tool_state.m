function [] = tool_state(state)
%1 -> ON | 0 -> OFF
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    toolStateMsg.Data = [state]; % Send 1 for on and 0 for off 
    send(toolStatePub,toolStateMsg);
end

