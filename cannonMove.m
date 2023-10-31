
function cannonMove(robot,cannon,endAngle)
            

    
    finalPos = cannon.getPose() * troty(endAngle)  ;                    % where the cannon will finish 
    finalPos = finalPos * transl(0.5,0,0)*trotx(pi/2)*troty(-pi/2);   %pose required for the gripper to be attached to canon 
    
     r = robot           % assign robot model 

     % get angle of the canon 
     rot = cannon.getPose();
    rotm = [rot(1:3,1:3)];
    rpy = rotm2eul(rotm);
    startAngle  = rpy(2);
    angleDifference = startAngle-endAngle;

    cannonPosition = [rot(1,4),rot(2,4),rot(3,4)];

    % get start and end positions 
    tr1 = FindCannonHandle(cannon)*transl(0.1,0,0)*trotx(pi/2)*troty(-pi/2);  % initial pose of robot given
    x1 = tr1(4,1);
    y1 = tr1(4,2);
    z1 = tr1(4,3);


    tr2 = finalPos
    x2 = tr2(4,1);
    y2 = tr2(4,2);
    z2 = tr2(4,3);

    % 1.1) Set parameters for the simulation
 
    t = 5;              % Total time (s)
    deltaT = 0.02;      % Control frequency
    steps = t/deltaT;   % No. of steps for simulation
    epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
    W = diag([0.5 0.5 0.5 1 0.1 0.1]);    % Weighting matrix for the velocity vector

    % 1.2) Allocate array data
    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,6);       % Array for joint anglesR
    qdot = zeros(steps,6);          % Array for joint velocities
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    x = zeros(3,steps);             % Array for x-y-z trajectory

   

    
   %between the two angles the value xyz will depend on the angle 


    % 1.3) Set up trajectory, initial pose
    s = lspb(0,1,steps)*angleDifference;              % Trapezoidal trajectory scalar, of the angle 
    
    for i=1:steps                                                           
        transpose = cannon.getPose() * troty(s(i));                       
        transpose = transpose * transl(0.5,0,0)*trotx(pi/2)*troty(-pi/2);       % where the canon will be for a given angle 
        
        x(1,i) = transpose(1,4);            % Points in x
        x(2,i) = transpose(2,4);            % Points in y
        x(3,i) = transpose(3,4);            % Points in z
        
        roti = transpose;
        rotmi = [roti(1:3,1:3)];
        rpyi = rotm2eul(rotmi);
        
        theta(1,i) = rpyi(3);                       % Roll angle 
        theta(2,i) = rpyi(2);                  % Pitch angle
        theta(3,i) = rpyi(1);                         % Yaw angle
    end
    
    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    q0 = deg2rad([45 -72 72 -72 -72 0]);                                                            % Initial guess for joint angles
    qMatrix(1,:) = r.model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint
    
    % 1.4) Track the trajectory with RMRC
    for i = 1:steps-1
        % UPDATE: fkine function now returns an SE3 object. To obtain the 
        % Transform Matrix, access the variable in the object 'T' with '.T'.
        T = r.model.fkine(qMatrix(i,:)).T;                                           % Get forward transformation at current joint state
        deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric!
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
        xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
        J = r.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
        qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
        for j = 1:6                                                             % Loop through joints 1 to 6
            if qMatrix(i,j) + deltaT*qdot(i,j) < r.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > r.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
       
    end

   
    for i = 1 : length(qMatrix)

         % movement for cannon based on arm movement 
        moveCannonTr = r.model.fkine(qMatrix(i,:)).T*troty(pi/2);
        rotm2 = [moveCannonTr(1:3,1:3)];
        rpy2 = rotm2eul(rotm2)
        angle2(i)  = rpy2(2);


        %movement for grippers 
    end
        




    % 1.5) Plot the results
    tic
    
    figure(1)
    plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
    for i = 1 : length(qMatrix)
    r.model.animate(qMatrix(i,:));
    cannon.move(transl(cannonPosition)*troty(angle2(i)));
    pause(0.01)

    end
    disp(['Plot took ', num2str(toc), 'seconds'])

         
            
end
