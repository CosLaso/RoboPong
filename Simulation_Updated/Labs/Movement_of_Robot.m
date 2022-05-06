%% Robotics
% Lab 4 - Question 3 & 4 - Inverse Kinematics & Joint Interpolation
function Lab4Solution_Question2and3

close all
clc

%% Options
interpolation = 2;                                                          % 1 = Quintic Polynomial, 2 = Trapezoidal Velocity
steps = 50;                                                                % Specify no. of steps

%% Load Model
mdl_puma560                                                                 
qlim = p560.qlim;                                                           

%% Define End-Effector transformation, use inverse kinematics to get joint angles
T1 = transl(0.5,-0.4,0.5);                                                  % Create translation matrix (from)
q1 = p560.ikine(T1);                                                        % Derive joint angles for required end-effector transformation
T2 = transl(0.5,0.4,0.1);                                                   % Define a translation matrix (to)       
q2 = p560.ikine(T2);                                                        % Use inverse kinematics to get the joint angles

%% Interpolate joint angles, also calculate relative velocity, accleration
qMatrix = jtraj(q1,q2,steps);
switch interpolation
    case 1
        qMatrix = jtraj(q1,q2,steps);
    case 2
        s = lspb(0,1,steps);                                             	% First, create the scalar function
        qMatrix = nan(steps,6);                                             % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
            end
    otherwise
        error('interpolation = 1 for Quintic Polynomial, or 2 for Trapezoidal Velocity')
end
        
velocity = zeros(steps,6);
acceleration  = zeros(steps,6);
for i = 2:steps
    velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);                          % Evaluate relative joint velocity
    acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
end

%% Plot the results
figure(1)
p560.plot(qMatrix,'trail','r-')                                             % Plot the motion between poses, draw a red line of the end-effector path

figure(2)
for i = 1:6
    subplot(3,2,i)
    plot(qMatrix(:,i),'k','LineWidth',1)
    title(['Joint ', num2str(i)])
    xlabel('Step')
    ylabel('Joint Angle (rad)')
    refline(0,qlim(i,1))                                                    
    refline(0,qlim(i,2))
end

figure(3)
for i = 1:6
    subplot(3,2,i)
    plot(velocity(:,i),'k','LineWidth',1)
    title(['Joint ', num2str(i)])
    xlabel('Step')
    ylabel('Joint Velocity')
end

figure(4)
for i = 1:6
    subplot(3,2,i)
    plot(acceleration(:,i),'k','LineWidth',1)
    title(['Joint ', num2str(i)])
    xlabel('Step')
    ylabel('Joint Acceleration')
end