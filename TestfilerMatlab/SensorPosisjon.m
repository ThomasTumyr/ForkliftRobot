clear; clc; close all;
import ETS3.*
L(1) = Link('revolute', 'd', 1.0,   'a', 0,    'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,     'a', 1.7,  'alpha', 0);
L(3) = Link('revolute', 'd', 0,     'a', 1.7,  'alpha', pi/2);
L
%Endre DH parametere utifra tabell
robot = SerialLink(L, 'name', '3DOF_Robot');

robot.plot([0 0 0]);
T_BS = transl(0.3, 0, 0.3);   % 30 cm forward, 0 cm right, 30 cm up
%Endre verdiene over til der sensor er iforhold til robot

% Object detected in sensor frame
T_SP = transl(0.5, 0.4, 0); % Kordinatene til objektet fra sensor
T_BP = T_BS * T_SP;
disp(T_BP)

q_goal = robot.ikine(T_BP, 'mask', [1 1 1 0 0 0]);  % position-only IK
disp(rad2deg(q_goal))

robot.plot(q_goal);
trplot(T_BS, 'frame', 'S', 'color', 'b');
trplot(T_BP, 'frame', 'P', 'color', 'r');