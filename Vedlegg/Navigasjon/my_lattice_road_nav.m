function lp = my_lattice_road_nav(init, goal)
%MY_LATTICE_ROAD_NAV  Lattice planning demo using Peter Corke's Robotics Toolbox

startAngle = 0; % Vinkel for start
stopAngle = pi; % Vinkel for slutt 
close all; clc; 
if nargin < 2
    init = [60 5 startAngle];   % start
    goal = [30 60 stopAngle];   % stopp
end

%% Map
road = zeros(100, 100); % Kart 100m^2  
road(:, 1:5) = 1;         % Venstre vegg
road(:, 95:100) = 1;      % Høyre vegg
road(15:80, 23:27) = 1;   % Hylle Venstre
road(15:80, 48:52) = 1;   % Hylle midt
road(15:80, 73:77) = 1;   % Hylle høyre
road(95:100,:) = 1;       % Topp vegg
road = logical(road);

%% Lattice planner (larger depth)
lp = Lattice(road, 'grid', 5, 'root', [50 10 0], 'depth', 6);
lp.plan();

figure; lp.plot();

%% Original path
disp('Original path:')
lp.query(init, goal);
lp.plot();

%% Increased turn cost
figure;
disp('Path with increased turn cost:')
lp.plan('cost',[1 10 10]);
lp.query(init, goal);
lp.plot();

end