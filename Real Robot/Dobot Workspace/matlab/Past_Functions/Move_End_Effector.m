function [] = Move_End_Effector(target)
%Move end effector to input coordinates
    [targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

    targetEndEffectorMsg.Position.X = target(1);
    targetEndEffectorMsg.Position.Y = target(2);
    targetEndEffectorMsg.Position.Z = target(3);
    
    %Write target rotation to message
    target_rotation = [0,0,0];
    
    qua = eul2quat(target_rotation);
    targetEndEffectorMsg.Orientation.W = qua(1);
    targetEndEffectorMsg.Orientation.X = qua(2);
    targetEndEffectorMsg.Orientation.Y = qua(3);
    targetEndEffectorMsg.Orientation.Z = qua(4);

    send(targetEndEffectorPub,targetEndEffectorMsg);
end

