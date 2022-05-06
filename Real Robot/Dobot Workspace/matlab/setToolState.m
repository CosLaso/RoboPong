function [] = setToolState(state)

% For Gripper 0 -> Released (OFF) | 1 -> Gripped (ON)
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    toolStateMsg.Data = [1 state];

end

