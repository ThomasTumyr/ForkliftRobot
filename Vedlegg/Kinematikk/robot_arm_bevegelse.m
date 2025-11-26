%% Kinematikk og simulering av arm

clc; clear all; close all;
import ETS3.*

global arm qn end_effector_pose end_effector_pose_matrix q1 q2 q3 q4 q5;
syms a b c d e

% Leddvinkler i standardposisjon
q1 = 0; q2 = pi/3; q3 = -pi/1.5; q4 = pi/3; q5 = 0;

% Lenge på links
L1 = 20; L2 = 170; L3 = 170; L4 = 20; L5 = 50;

% DH-parametere
L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
L(4) = Link('revolute', 'd', 0, 'a', L4, 'alpha', -pi/2);
L(5) = Link('revolute', 'd', 0, 'a', L5, 'alpha', pi/2);
arm = SerialLink(L,'name',' ')

% Setter limits på leddene
arm.qlim = [[-pi/2 pi/2]; [0 pi/2]; [-pi/1.25 pi/2]; [-pi/2 pi/2]; [-pi/4 pi/4]];
% Setter en workspace for armen
W = [-500 500 -500 500 -500 500];

% Lager en array med leddvinkler
qn = [q1 q2 q3 q4 q5];
format short;
end_effector_pose = arm.fkine(qn);
end_effector_pose_matrix = end_effector_pose.T;

% Sjekker dimensjoner på end_effector_pose
if isnumeric(end_effector_pose_matrix) && all(size(end_effector_pose_matrix) == [4 4])
    disp('end_effector_pose_matrix er en 4x4 matrise:');
    disp(end_effector_pose_matrix);
else
    disp('Feil: end_effector_pose_matrix er ikke en 4x4 matrise!');
    disp('Det returnerte resultatet er:');
    disp(end_effector_pose_matrix);
end

%arm.plot(qn,'jvec');
E = arm.fkine([a b c d e]);
E_T = E.T;
E_T = vpa(E_T,2) % Generell transformasjonsmatrise

format short e;
ehh = E_T(1:4,1);
ehh2 = E_T(1:4,2);
ehh3 = E_T(1:4,3);
ehh4 = E_T(1:4,4);

ehhsubs = subs(E_T,[a b c d e],[q1 q2 q3 q4 q5]);
ehhsubs = vpa(ehhsubs,3) % For å sjekke ekvivalens

%% Differential kinematics
global qd ;
% Setter leddhastigheter (for eksempel, alle ledd beveger seg med 1 rad/s)
qd = [1; 1; 1; 1; 1]; % Leddhastigheter i radianer per sekund
J = arm.jacob0(qn) % Finner jacobi matrisen

% Separerer ut translasjons- og rotasjonsdeler
Jv = J(1:3, :);  % Translasjonsdel
Jw = J(4:6, :);  % Rotasjonsdel
% Beregner de totale hastighetene til endeeffekten
v = Jv * qd;     % Translasjons hastighet
w = Jw * qd;     % Rotasjons hastighet
% Vis resultatene
disp('Translasjons hastighet (v):');
disp(v);
disp('Rotasjons hastighet (w):');
disp(w);

% Henter ut posisjonen fra end_effector_pose
x_pos = end_effector_pose_matrix(1, 4); % X-posisjon
y_pos = end_effector_pose_matrix(2, 4); % Y-posisjon

% Visualiser translasjons hastighet vektor på roboten
hold on;
quiver(x_pos, y_pos, v(1), v(2), 'r', 'LineWidth', 2, 'MaxHeadSize', 1, 'AutoScale', 'on');
axis equal; grid on;
title('Translasjons hastighet ved endeeffekten');
xlabel('X');
ylabel('Y');

%% Forward kinematics
clc;
T = arm.fkine([q1 q2 q3 q4 q5]);
T = vpa(T,3)
% Generell formel
% T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
%     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
%     0 sin(alpha) cos(alpha) d
%     0 0 0 1]

% T0_1
theta = q1;
d = 20;
a = 0;
alpha= sym(pi/2);

T0_1 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
    0 sin(alpha) cos(alpha) d
    0 0 0 1];

T0_1 = vpa(T0_1,3)

% T1_2
theta = q2;
d = 0;
a = 170;
alpha= 0;

T1_2 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
    0 sin(alpha) cos(alpha) d
    0 0 0 1];

T1_2 = vpa(T1_2,3)

% T2_3
theta = q3;
d = 0;
a = 170;
alpha= 0;

T2_3 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
    0 sin(alpha) cos(alpha) d
    0 0 0 1];

T2_3 = vpa(T2_3,3)

% T3_4
theta = q4;
d = 0;
a = 20;
alpha= -sym(pi/2);

T3_4 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
    0 sin(alpha) cos(alpha) d
    0 0 0 1];

T3_4 = vpa(T3_4,3)

% T4_5
theta = q5;
d = 0;
a = 50;
alpha= sym(pi/2);

T4_5 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
    0 sin(alpha) cos(alpha) d
    0 0 0 1];

T4_5 = vpa(T4_5,3)

T_04 = T0_1*T1_2*T2_3*T3_4;
T_04 = simplify(T_04);
T_04 = vpa(T_04,4)

T_05 = T0_1*T1_2*T2_3*T3_4*T4_5;
T_05 = simplify(T_05);
T_05 = vpa(T_05,4)

T_tool = arm.fkine(qn).T % toolbox FK
difference = norm(T_tool - T_05); 
disp(difference); %Forskjellen mellom fkine og manuell utregning blir omtrent 0 

difference2 = norm(T_tool - T_04); 
disp(difference2); %Forskjellen blir omtrent 50, dette er fordi den siste linken med lengde 50 ikke er tatt med i utregningen


%% Inverse kinematics
clc;
global TA mask;
mask = [1 1 1 0 1 1]
TA = arm.fkine([qn(1) qn(2) qn(3) pi/2.99999 qn(5)]) % Får ikke lov å bruke pi/3
%TA = arm.fkine(qn) 

invKin = arm.ikine(TA, 'mask', mask)
armINV = arm.fkine(invKin) % samme posisjon på end-effector
disp(end_effector_pose)
arm.plot(invKin)%,'movie','C:\Users\oskar\OneDrive - Høgskulen på Vestlandet\Skole\5.semester\ELE306\')

%% Simulering av arm
clc;
global num_points q_start q_mid q_mid2 q_mid3 q_end total_num_points;

% Antall ønskede punkter
num_points = 50;  % Kan endre dette antallet etter behov
total_num_points = num_points * 4; % totalt antall punkter for alle segmenter

% Startposisjon
q_start = [0, pi/2.5, -pi/1.25, pi/2.5, 0];
% Mellompunkt 1
q_mid = [0, pi/6, -pi/3, pi/6, 0];
% Mellompunkt 2
q_mid2 = [0, pi/3, -pi/1.5, pi/3, 0];
% Mellompunkt 3
q_mid3 = [pi/3, pi/3, -pi/6, -pi/6, -pi/6];
% Sluttposisjon
q_end = [0, pi/2.5, -pi/1.25, pi/2.5, 0];

% Lineær interpolasjon mellom start- og sluttposisjoner
traj = zeros(total_num_points, numel(q_start));
% Bevegelse fra q_start til q_mid (kartesisk interpolering)
end_effector_start = arm.fkine(q_start);
end_effector_mid = arm.fkine(q_mid);

end_effector_start_matrix = end_effector_start.T;
end_effector_mid_matrix = end_effector_mid.T;
% Sjekk dimensjoner
disp("Size of End Effector Start Matrix:");
disp(size(end_effector_start_matrix)); % Forventet: [4, 4]
disp("Size of End Effector Mid Matrix:");
disp(size(end_effector_mid_matrix));   % Forventet: [4, 4]


if isnumeric(end_effector_start_matrix) && all(size(end_effector_start_matrix) == [4 4])
    disp('end_effector_pose er en 4x4 matrise:');
    disp(end_effector_start_matrix);
else
    disp('Feil: end_effector_pose er ikke en 4x4 matrise!');
    disp('Det returnerte resultatet er:');
    disp(end_effector_start_matrix);
end

%Kjør lineær interpolasjon
for i = 1:num_points
    alpha = (i - 1) / (num_points - 1);
    cartesian_position = (1 - alpha) * end_effector_start_matrix(1:3, 4)' + ...
                         alpha * end_effector_mid_matrix(1:3, 4)';
    % Bruk initial guess for q
    %initial_guess = q_start; % eller en annen fornuftig verdi
    joint_angles = arm.ikine(transl(cartesian_position), 'mask', mask, 'q0', q_start);
    
    % Sjekk om joint_angles er tom
    if isempty(joint_angles)
        disp('Feil: Ingen løsning funnet for interpolert posisjon.');
    else
        traj(i, :) = joint_angles; % Lagre leddvinkler
    end
end

% Bevegelse fra q_mid til q_mid2 (interpolasjon)
for i = 1:num_points
    alpha = (i - 1) / (num_points - 1);
    traj(num_points + i, :) = q_mid + (q_mid2 - q_mid) * alpha;
end

% Bevegelse fra q_mid2 til q_mid3 (interpolasjon)
for i = 1:num_points
    alpha = (i - 1) / (num_points - 1);
    traj(2*num_points + i, :) = q_mid2 + (q_mid3 - q_mid2) * alpha;
end

% Bevegelse fra q_mid3 til q_end (interpolasjon)
for i = 1:num_points
    alpha = (i - 1) / (num_points - 1);
    traj(3*num_points + i, :) = q_mid3 + (q_end - q_mid3) * alpha;
end

% Plote armen
arm.plot(traj,'trail',{'r', 'LineWidth', 2});

%% Lagre video
%Definer filbanen

folderPath = 'C:\Users\oskar\OneDrive - Høgskulen på Vestlandet\Skole\5.semester\ELE306\'; % Angi stien til mappen
fileName = 'file2.mp4';              % Angi filnavnet
fullPath = fullfile(folderPath, fileName); % Konstruksjon av full filbane
% Sjekk om filen eksisterer
if exist(fullPath, 'file') == 2
    disp('Filen finnes.');
else
    disp('Filen finnes ikke.');
    arm.plot(traj,'movie',fullPath,'trail',{'r', 'LineWidth', 2});
end
