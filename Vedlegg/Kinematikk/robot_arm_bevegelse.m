
% Kinematikk og simulering av arm

clc; clear all; close all;
import ETS3.*

syms q1 q2 q3 q4 q5

% DH - Parameter
L1 = 20;
L2 = 170;
L3 = 170;
L4 = 50;
L5 = 10;

L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', -pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
L(4) = Link('revolute', 'd', 0, 'a', L4, 'alpha', pi/2);
L(5) = Link('revolute', 'd', 0, 'a', L5, 'alpha', pi/2);
arm = SerialLink(L,'name', 'open manipulator')


% Forward kinematics
T = arm.fkine([q1 q2 q3 q4 q5]);

T = vpa(T,3);


%arm.teach

% Kopiert fra 'T', fordi svaret ble for langt, og kunne ikke bruke vpa()
a = [0.5*cos(q1 + q2 + q3 + q4) + 0.5*cos(q2 - q1 + q3 + q4), - 0.5*sin(q1 + q2 + q3 + q4) - 0.5*sin(q2 - q1 + q3 + q4),                                            sin(q1), 2.0*cos(q1)*(36.0*cos(q2 + q3 + q4) + 60.0*cos(q2 + q3) - 85.0*sin(q2))];
b = [0.5*sin(q1 + q2 + q3 + q4) - 0.5*sin(q2 - q1 + q3 + q4),   0.5*cos(q1 + q2 + q3 + q4) - 0.5*cos(q2 - q1 + q3 + q4),                                       -1.0*cos(q1), 2.0*sin(q1)*(36.0*cos(q2 + q3 + q4) + 60.0*cos(q2 + q3) - 85.0*sin(q2))];
c = [                                      sin(q2 + q3 + q4),                                         cos(q2 + q3 + q4), 0.0000,      72.0*sin(q2 + q3 + q4) + 120.0*sin(q2 + q3) + 170.0*cos(q2) + 40.5];
d = [                                                      0,                                                         0,                                                  0,                                                                     1.0];
 

TT = vpa([a;b;c;d],4)

% Differensial kinematikk
W0 = [0; 0; 1.962; 0; 0; 0];

J0 = arm.jacob0([0 100 50 -50 0])
Q = J0'*W0


qn = deg2rad([0 100 50 -50 0])

% Invers kinematikk
TA = arm.fkine(qn);

invKin = arm.ikine(TA, 'mask', [1 1 1 1 0 0])


% Simulering av arm
% Vektrelaterte parametere


% Opprett en robot fra Toolbox
% Vektrelaterte parametere
arm_mass = 1000;  % Vekt av robotarmen i gram
gripper_mass = 200;  % Vekt av gripperen i gram
camera_mass = 100;  % Vekt av kameraet i gram
toy_mass = randi([0, 500]);
max_payload = 1800;  % Maksimal tillatt nyttelast i gram

% Skriv ut størrelsen på leken
fprintf('Størrelsen på leken: %d gram\n', toy_mass);

% Kombinert vekt av robotarmen, gripperen og kameraet
total_mass = toy_mass + arm_mass + gripper_mass + camera_mass;

% Sjekk om totalvekten overskrider den maksimale nyttelasten
if total_mass > max_payload
    disp('Totalvekten overskrider den maksimale nyttelasten. Juster vektene.');
else
    % Antall ønskede punkter
num_points = 50;  % Du kan endre dette antallet etter behov

% Startposisjon
q_start = [0, -pi/4, pi/2, -pi/4, 0];

% Mellompunkt
q_mid = [0, -pi/2, pi/4, pi/4, 0];

% Sluttposisjon
q_end = [pi/2, -pi/3, pi/6, pi/6, 0];

% Lineær interpolasjon mellom start- og sluttposisjoner
traj = zeros(num_points, numel(q_start));

for i = 1:num_points
    % Lineær interpolasjon for hvert ledd
    for j = 1:numel(q_start)
        if i <= num_points/2
            traj(i, j) = q_start(j) + (q_mid(j) - q_start(j)) * (i - 1) / (num_points/2 - 1);
        else
            traj(i, j) = q_mid(j) + (q_end(j) - q_mid(j)) * (i - num_points/2 - 1) / (num_points/2 - 1);
        end
    end
end

% Plot bevegelsen
figure;
arm.plot(traj);

end
