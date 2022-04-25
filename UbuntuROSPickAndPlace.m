%pick and place ubuntu ROS
%https://au.mathworks.com/help/robotics/ug/pick-and-place-workflow-in-gazebo-using-ros.html
%In VM settings, VM > Settings > Hardware > Display, disable Accelerate 3D graphics.
%Start the Ubuntu® virtual machine desktop.
%In the Ubuntu desktop, click the Gazebo Recycling World icon to start the Gazebo world built for this example.
%Specify the IP address and port number of the ROS master in Gazebo so that MATLAB® can communicate with the robot simulator. For this example, the ROS master in Gazebo uses the IP address of 192.168.203.131 displayed on the Desktop. Adjust the rosIP variable based on your VM.
%Start the ROS 1 network using rosinit.

rosIP = '192.168.203.131';   % IP address of ROS-enabled machine  

rosinit(rosIP,11311); % Initialize ROS connection

%This example uses a Stateflow chart to schedule tasks in the example. Open the chart to examine the contents and follow state transitions during chart execution.
edit exampleHelperFlowChartPickPlaceROSGazebo.sfx

%This simulation uses a KINOVA Gen3 manipulator with a Robotiq gripper attached. Load the robot model from a .mat file as a rigidBodyTree object.
load('exampleHelperKINOVAGen3GripperROSGazebo.mat'); 

%Set the initial robot configuration. Create the coordinator, which handles the robot control, by giving the robot model, initial configuration, and end-effector name.
initialRobotJConfig =  [3.5797   -0.6562   -1.2507   -0.7008    0.7303   -2.0500   -1.9053];
endEffectorFrame = "gripper";

%Initialize the coordinator by giving the robot model, initial configuration, and end-effector name.
coordinator = exampleHelperCoordinatorPickPlaceROSGazebo(robot,initialRobotJConfig, endEffectorFrame);

%Specify the home configuration and two poses for placing objects.
coordinator.HomeRobotTaskConfig = getTransform(robot, initialRobotJConfig, endEffectorFrame); 
coordinator.PlacingPose{1} = trvec2tform([[0.2 0.55 0.26]])*axang2tform([0 0 1 pi/2])*axang2tform([0 1 0 pi]);
coordinator.PlacingPose{2} = trvec2tform([[0.2 -0.55 0.26]])*axang2tform([0 0 1 pi/2])*axang2tform([0 1 0 pi]);

%Connect the coordinator to the Stateflow Chart. Once started, the Stateflow chart is responsible for continuously going through the states of detecting objects, picking them up and placing them in the correct staging area.
coordinator.FlowChart = exampleHelperFlowChartPickPlaceROSGazebo('coordinator', coordinator); 

%Use a dialog to start the pick-and-place task execution. Click Yes in the dialog to begin the simulation.
answer = questdlg('Do you want to start the pick-and-place job now?', ...
         'Start job','Yes','No', 'No');

switch answer
    case 'Yes'
        % Trigger event to start Pick and Place in the Stateflow Chart
        coordinator.FlowChart.startPickPlace;       
    case 'No'
        coordinator.FlowChart.endPickPlace;
        delete(coordinator.FlowChart)
        delete(coordinator);
end

%The Stateflow chart will finish executing automatically after 3 failed attempts to detect new objects. To end the pick-and-place task prematurely, uncomment and execute the following lines of code or press Ctrl+C in the command window.
% coordinator.FlowChart.endPickPlace;        
% delete(coordinator.FlowChart);
% delete(coordinator);

if strcmp(answer,'Yes')
    while  coordinator.NumDetectionRuns <  4
        % Wait for no parts to be detected.
    end
end

%Shutdown the ROS network after finishing the example.
rosshutdown
