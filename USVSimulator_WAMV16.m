%% Clear
close all ;
clear ;
clc ;

%% Specification of USV
% Size
B = 2.5 ;     % Breath (m)

% Coefficients for state-space equation
C1 = [- 1.0191, 0, 0 ;
    0, 0.0161, - 0.0052 ;
    0, 8.2861, - 0.9860] ;
C2 = [0.0028, 0 ;
    0, 0.0002 ;
    0, 0.0307] ;

%% Conditions for simulation
t_start = 0 ;     % Start time for simulation(sec)
t_end = 10 ;      % End time for siumlation(sec)
dt = 0.01 ;    % Time sep interval(sec)
t = t_start:dt:t_end ;      % Time step array

%% Initial condition for running the simulation
% Initial condition
x = 0 ;     % x position in Body-fixed coordinates(m)
y = 0 ;     % y position in Body-fixed coordinates(m)
psi = 0 ;   % Bearing in Body-fixed coordinates(rad)
u = 0 ;     % x velocity in Body-fixed coordinates(m/s)
v = 0 ;     % y velocity in Body-fixed coordinates(m/s)
r = 0 ;     % Anglular velocity in Body-fixed coordinates(rad/sec)
eta = [x ;
       y ;
       r] ;      % Position matrix
nu = [u ;
      v ;
      r] ;  % Velocity matrix
  
% RPM
rpm_port = 1000 ;    % RPM of port thruster
rpm_stbd = 500 ;  % RPM of starboard thruster

% RPM to thrust(Only forward)
T_port = 3.54 * 1e-5 * (rpm_port^2) + 0.084 * rpm_port - 3.798 ;    % Thrust from port(N)
T_stbd = 3.54 * 1e-5 * (rpm_stbd^2) + 0.084 * rpm_stbd - 3.798 ;    % Thrust from starboard(N)
tau_X = T_port + T_stbd ;       % x thrust in Body-fixed coordinates(N)
tau_N = (T_port - T_stbd) * B / 2 ;     % Angular thrust in Body-fixed coordinates(N*m)
f = [tau_X ;
     tau_N ] ;      % Thrust matrix

%% Running the simulation
% Initial position in Earth-fixed coordinates
X_E = 0 ;       % (m)
Y_E = 0 ;       % (m)
PSI_E = 0 ;     % (rad)

% Calculate velocity matrix and position matrix for steps going
for i = 1:length(t)
    %% Calculate the accelerations for next step in Body-fixed coordinates
    nu_dot = C1 * nu + C2 * f ; 
    
    %% Calculate the velocities for next step in Body-fixed coordinates
    nu = nu + nu_dot * dt ;
    
    %% Calculate the positions for next step in Body-fixed coordintates
    eta = eta + nu * dt ;
    
    %% Convert Body-fixed position to Earth-fixed position
    % Unpack the position matrix
    x = eta(1) ;
    y = eta(2) ;
    psi = eta(3) ;
    
    % Separate the Body-fixed positions into Earth-fixed positions
    x_Xi = x * cos(psi) ;
    x_Yi = x * sin(psi) ;
    y_Xi = - y * sin(psi) ;
    y_Yi = y * cos(psi) ;
    
    % Reunite the positions in Earth-fixed coordinates 
    X = x_Xi + y_Xi ;   
    Y = x_Yi + y_Yi ;
    
    %% Make it start from the last position
    X_E = X_E + X ;
    Y_E = Y_E + Y ;
    PSI_E = PSI_E + psi ;
    
    %% Keep the position history to plot
    history_X_E(i) = X_E ;
    history_Y_E(i) = Y_E ;
end
    
%% Plot the position history ;
figure(1) ;
plot(history_X_E, history_Y_E) ;
pbaspect([1 1 1]) ;
daspect([1 1 1]) ;
hold on ;
grid on ;
xlabel('x_i(m)') ;
ylabel('y_i(m)') ;
