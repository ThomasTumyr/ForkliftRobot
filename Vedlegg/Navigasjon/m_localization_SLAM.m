function void=m_localization_SLAM()
% Simulat Simultaneous Localization and Mapping

%% Initializing
randinit();                             % Initializes the radom number generator

%% Navigation with a map
map = LandmarkMap(20)                   % Create a landmark object
map.plot();                             % Plot the map with landmarks

%% Vehicle model
V = diag([0.02, 0.5*pi/180].^2);        % Specify process covariance (distance/heading)
veh = Bicycle('covar',V);               % Creates a vehicle from the bycycle model with covariance V
veh.add_driver( RandomPath(map.dim) );  % Adds driver that steers robot to random WPs

%% Sensor measurments
W = diag([0.1, 1*pi/180].^2);           % Specify the sensor covariance matrix
sensor = RangeBearingSensor(veh, map,'covar',W);    
                                        % Create sensor object with covariance 

%% Kalman filter
P0 = diag( [0.01, 0.01, 0.005].^2 );    % Inital covarance matrix
ekf = EKF(veh,V,P0, sensor,W,[]);       % Create Kalman filter object ([]) means map is unknown
ekf.run(1000);                          % Runs the EKF for 1000 time steps

%% Plotting the results
veh.plot_xy('b');                       % Plotting vehicle trajectory
ekf.plot_map('g');                      % Plotting estimated trajectory
ekf.plot_xy('r');                       % Plotting estimated landmarks
ekf.plot_ellipse();                     % Plotting error ellipses
