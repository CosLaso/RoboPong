function [qMatrix] = RMRCMovement(robot,fPoint)    % Adapted from the RMRC_2 file

    lims = robot.jointLimits;
    iPoint = robot.model.fkine(robot.model.getpos());
    iPoint = iPoint(1:3,4);

    t = 5;              % Total time (s)
    deltaT = 0.05;      % Control frequency
    steps = t/deltaT;   % No. of steps for simulation
    epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1]);  % Weighting matrix for the velocity vector

    % 1.2) Allocate array data
    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,3);       % Array for joint angles
    qdot = zeros(steps,3);          % Array for joint velocities
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    x = zeros(3,steps);             % Array for x-y-z trajectory
    
    % 1.3) Set up trajectory, initial pose
    s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
    for i=1:steps
        x(1,i) = (1-s(i))*iPoint(1)+s(i)*fPoint(1); % Points in x
        x(2,i) = (1-s(i))*iPoint(2)+s(i)*fPoint(2); % Points in y
        x(3,i) = (1-s(i))*iPoint(3)+s(i)*fPoint(3); % Points in z
        theta(1,i) = 0; % Roll angle
        theta(2,i) = 0; % Pitch angle
        theta(3,i) = 0; % Yaw angle
    end

    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    q0 = zeros(1,3);                                                            % Initial guess for joint angles
    qMatrix(1,:) = robot.model.ikcon(T,q0);                                     % Solve joint angles to achieve first waypoint

    % Track the trajectory with RMRC
    for i = 1:steps-1
        T = robot.model.fkine(qMatrix(i,:));                                    % Get forward transformation at current joint state
        deltaX = x(:,i+1) - T(1:3,4);                                       	% Get position error from next waypoint
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
    end

    robot.model.plot(qMatrix,'trail','r-')

end

