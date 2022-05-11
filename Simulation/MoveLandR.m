function MoveLandR(Dobot, initialJoints, direction)

    newPosition = transl(direction);

    steps = 50;

    targJointAngles = Dobot.model.ikine(newPosition, initialJoints, [1,1,1,0,0,0]);
    jointTrajectory = jtraj(initialJoints, targJointAngles, steps);                 % (Current q, target q, steps)
    for i = 1:steps
        animate(Dobot.model,jointTrajectory(i,:));
        drawnow();
    end
end
%     % Go from ibrick to start
%     currJointAngles = targJointAngles
%     jointTrajectory = jtraj(currJointAngles, initialJoints, steps);
%     for i = 1:steps
%         animate(Dobot.model,jointTrajectory(i,:));
%         drawnow();
%     end