function [qMatrix] = MoveWithRMRC(robot,fPoint,traj)    % Adapted from the RMRC_2 file

    lims = robot.jointLimits;

    iPoint = robot.model.fkine(robot.model.getpos());
    iPoint = iPoint(1:3,4);

    % Set parameters
    t = 5;                  % Total time (s)
    deltaT = 0.05;          % Control frequency
    steps = t/deltaT;       % No. of steps for simulation i.e. steps = t/deltaT
    delta = 2*pi/steps;     % Small angle change
    epsilon = 0.01;         % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1]);      % Weighting matrix for the velocity vector

    % Allocate array data
    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,5);       % Array for joint anglesR
    qdot = zeros(steps,3);          % Array for joint velocities
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    xyz = zeros(3,steps);             % Array for x-y-z trajectory

    % Set up trajectory, initial pose
    s = lspb(0, 1, steps);          % Trapezoidal trajectory scalar.
    switch traj
        case 1
            for i=1:steps
                xyz(1,i) = (1-s(i))*iPoint(1)+s(i)*fPoint(1); % Points in x
                xyz(2,i) = (1-s(i))*iPoint(2)+s(i)*fPoint(2); % Points in y
                xyz(3,i) = (1-s(i))*iPoint(3)+s(i)*fPoint(3); % Points in z
                theta(1,i) = 0; % Roll angle
                theta(2,i) = 0; % Pitch angle
                theta(3,i) = 0; % Yaw angle
            end
        case 2
            d2 = pi/steps;
            for i = 1:steps
                xyz(1,i) = (1-s(i))*iPoint(1)+s(i)*fPoint(1);
                xyz(2,i) = (1-s(i))*iPoint(2)+s(i)*fPoint(2);
                xyz(3,i) = iPoint(3)+ 0.05*sin(i*d2); % height = 0.05
                theta(1,i) = 0; % Roll angle
                theta(2,i) = 0; % Pitch angle
                theta(3,i) = 0; % Yaw angle
            end
        case 3
            for i = 1:steps
                d2 = pi/steps;
                dx = fPoint(1)-iPoint(1);
                dy = fPoint(2)-iPoint(2);
                d = (dx^2+dy^2)^0.5;
                h = dx/2+iPoint(1);
                k = dy/2+iPoint(2);
                theta = atan2(k,h);
                if theta < 0
                    theta = theta + 2 * pi;
                end
                xyz(1,i) = h+d/2*cos(pi/2+theta-delta*(i - 1));
                xyz(2,i) = k+d/2*sin(pi/2+theta-delta*(i - 1));
                xyz(3,i) = (1-s(i))*iPoint(3)+s(i)*fPoint(3);
                theta(1,i) = 0; % Roll angle
                theta(2,i) = 0; % Pitch angle
                theta(3,i) = 0; % Yaw angle
            end
    end
    qMatrix(1,:) = robot.model.getpos();                                            % Solve joint angles to achieve first waypoint

    % Track the trajectory with RMRC
    for i = 1:steps-1
        T = robot.model.fkine(qMatrix(i,:));                                    % Get forward transformation at current joint state
        deltaX = xyz(:,i+1) - T(1:3,4);                                       	% Get position error from next waypoint
        linear_velocity = (1/deltaT)*deltaX;
        xdot = W*linear_velocity;                                           	% Calculate end-effector velocity to reach next waypoint.
        J = robot.model.jacob0(qMatrix(i,:));                                   % Get Jacobian at current joint state
        J = J(1:3,1:3);
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1-m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J+lambda*eye(3))*J';                                      % DLS Inverse
        qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the vector)
        qlim = robot.model.qlim;
        for j = 1:3                                                             % Loop through joints 1 to 3
            if J == 3
                [~, index] = min(abs(lims(:,1)-qMatrix(i,2)));
                [~, index2] = min(abs(lims(:,3)-qMatrix(i,2)));
                qlim(3,1) = lims(index,2);
                qlim(3,2) = lims(index2,4);
            end 
            if qMatrix(i,j) + deltaT*qdot(i,j) < qlim(j,1)                     % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > qlim(j,2)                 % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end

        qMatrix(i+1,1:3) = qMatrix(i,1:3)+deltaT*qdot(i,:);                     % Update next joint state based on joint velocities
        qMatrix(i+1,4) =- (pi/2-qMatrix(i+1,2)-qMatrix(i+1,3));
        for i = 1:size(qMatrix,1)
            robot.model.animate(qMatrix(i,:));
            pause(0.05);
        end
    end

end

