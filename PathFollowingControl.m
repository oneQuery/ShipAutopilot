%% Initialize
clc ; clear ; close all ;

%% Ship specification
beam = 2.5 ;
thrustLimit = 800 ;

%% Initial ship condition
% Position in the Earth coordinate system
xEarth = 0 ;    
yEarth = 0 ;     
psi = 0 ;   

% Velocity in the body coordinate system 
uBody = 1 ;
vBody = 0 ;
r = 0 ;         % angular velocity is the same in the both coordinate system

% Velocity in the Earth coordinate system
uEarthX = uBody * cos(psi) ;        % Earth's x direction component of uBody
uEarthY = uBody * sin(psi) ;        % Earth's y direction component of uBody
vEarthX = - vBody * sin(psi) ;      % Earth's x direction component of vBody
vEarthY = vBody * cos(psi) ;        % Earth's y direction component of vBody

VEarthX = uEarthX + vEarthX ;       % Earth's x direction component of V(ship speed)
VEarthY = uEarthY + vEarthY ;       % Earth's y direction component of V(ship speed)

%% Waypoints in the Earth coordinate system
waypoint = [[3, 3];
            [10, 15]] ;

%% Condition for simulation
time = 0 ;      % initial time
dt = 0.1 ;      % time step interval
reach = 0.5 ;      % reach criterion
xEarthHistory = [] ;    % container to draw trajectory 
yEarthHistory = [] ;    % container to draw trajectory

%% Simulation running till the ship reaches the final destination
for waypointIndex = 1:length(waypoint)
    isReached = false ;     % bool to check the ship reached the waypoint
    
    % Feed waypoint one by one
    singleWaypoint = waypoint(waypointIndex, :) ;

    xWaypoint = singleWaypoint(1) ;     % unpack
    yWaypoint = singleWaypoint(2) ;     % unpack

    while ~isReached
        %% Advance direction angle error
        errorX = xWaypoint - xEarth ;        % error of x between the target and the ship
        errorY = yWaypoint - yEarth ;        % error of y between the target and the ship
        
        theta = atan2(errorY, errorX) ;      % angle of waypoint deriection in Earth coordinate system
        if theta < 0                         % compensate the angle representation range from [-pi, pi] to [0, 2*pi]
           theta = theta + 2*pi ;
        end
        
        errorN = theta - psi ;               % error of y between the target and the ship
        
        if pi <= errorN && errorN < 2*pi     % compensate the angle representation range from [-pi, pi] to [0, 2*pi]
            errorN = errorN - 2*pi ;
        elseif -2*pi < errorN && errorN < -pi
            errorN = errorN + 2*pi ;
        end
        
        %% P controller
        % input: angle error, output: control force
        pGain = 150 ;
        controlInputForce = pGain * errorN ;

        % Control force saturation. Control force(will be tau_N) is limited when it exceed the limit of thrust
        if controlInputForce > thrustLimit
            controlInputForce = (thrustLimit + thrustLimit) * beam / 2 ;
        elseif controlInputForce < - thrustLimit
            controlInputForce = -(thrustLimit + thrustLimit) * beam / 2 ;
        end

        %% Platform
        % input: control force, output: ship positoin
        % ** note that the surge is ingored in this probelm. (i.e., u is constant) **

        % Variables for the state-space equation of the ASV (Rank is reduced down due to the ingnored surge)
        systemMat = [0.0161, -0.0052; 8.2861, -0.9860] ;
        inputMat = [0.0002; 0.0307] ;
        velocityBodyMat = [vBody; r] ;     % Pack the ship velocity components
        tauN = controlInputForce ;

        % Ship acceleration from the state-space equation
        acclerationBodyMat = systemMat * velocityBodyMat + inputMat * tauN ;

        % New ship velocity from the ship acceleration
        velocityBodyMat = [uBody; 
                           velocityBodyMat + acclerationBodyMat * dt] ;      % velocity from the integration of acceleration

        uBody = velocityBodyMat(1) ;       % unpack
        vBody = velocityBodyMat(2) ;       % unpack
        r = velocityBodyMat(3) ;           % unapck

        % New ship position from the new ship velocity
        psi = psi + r * dt ;          
        psi = mod(psi, 2*pi) ;      % compensate the angle representation range from [0, inf] to [0, 2*pi]
                    
            % Transformation of the coordinate system form the body tp the Earth
        uEarthX = uBody * cos(psi) ;        % Earth's x direction component of uBody
        uEarthY = uBody * sin(psi) ;        % Earth's y direction component of uBody
        vEarthX = - vBody * sin(psi) ;      % Earth's x direction component of vBody
        vEarthY = vBody * cos(psi) ;        % Earth's y direction component of vBody

        VEarthX = uEarthX + vEarthX ;       % Earth's x direction component of V(ship speed)
        VEarthY = uEarthY + vEarthY ;       % Earth's y direction component of V(ship speed)

        xEarth = xEarth + VEarthX * dt ;
        yEarth = yEarth + VEarthY * dt ;

        %% Visualization
        % Genrate trajectory matrix
        xEarthHistory(end+1) = xEarth ;
        yEarthHistory(end+1) = yEarth ;

        shipPositionFig = figure(1) ;
        figure(shipPositionFig) ;

        % Draw
        plot(yEarthHistory, xEarthHistory, '.b') ;      % trajectory
        hold on ;
        plot(waypoint(:, 2), waypoint(:, 1), 'vr') ;              % waypoints
        quiver(yEarth, xEarth, VEarthY, VEarthX, 'm') ; % velocity vector
        quiver(yEarth, xEarth, errorY, errorX, 'c') ;   % the vector from the ship to destination
        hold off ;

        % Figure configuration
        title('ASV trajectory') ;
        xlabel('y(m)') ;
        ylabel('x(m)') ;
        grid on ;
        pbaspect([1 1 1])
        axis equal

        %% Check it reached waypoint 
        errorDistance = sqrt(errorX^2 + errorY^2) ;      
        if errorDistance < reach
            isReached = true ;
        end

        %% Record the time
        time = time + dt ;

        %% Print out the state
        fprintf('time: %.2f sec, distnace error: %.2f m\n', time, errorDistance') ;
    end
 end
disp('Reached the destination')