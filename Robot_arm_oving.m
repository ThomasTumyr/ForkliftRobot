%clear import
import ETS2.*
a1 = 0.4; a2 = 0.75; a3 = 0.5;
E = Rz('q1') * Tx(a1) * Rz('q2') * Tx(a2) * Rz('q3') * Tx(a3)
E.fkine([30,45,10],'deg')
E.plot([30,45,10],'deg')
%E.teach

%% 
%clear import
import ETS3.*
L1 = 1; L2 = 0; L3 = 2; L4 = 2;

E3 = Tz(L1) * Rz('q1') * Ry('q2') * Ty(L2) * Tz(L3) * Ry('q3') * Tx(L4) * Rz('q4')
E3.fkine([0, 0, 0, 0],'deg')
E3.teach

%% 
import ETS3.*
syms q1 q2 q3 q4
L1 = 1; L2 = 2; L3 = 2; L4 = 2;
L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', -pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', pi/2);
L(4) = Link('revolute', 'd', 0, 'a', L4, 'alpha', 0);
arm = SerialLink(L,'name', 'open manipulator')
arm.teach
vpa(arm.fkine([q1, q2, q3, q4],'deg'),3)

%% 1.i.2
clc; clear all; close all;

syms q1 q2 q3 q4

% DH parameter til transformasjonsmatrise

% Bruk DH parameterene

% Generell formel, j-1 T j
% T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
%     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
%     0 sin(alpha) cos(alpha) d
%     0 0 0 1]; 

% T0_1
theta = q1;
d = 20;
a = 0;
alpha= -sym(pi/2);

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
alpha= sym(pi/2);

T2_3 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
    0 sin(alpha) cos(alpha) d
    0 0 0 1];

T2_3 = vpa(T2_3,3)

% % T3_4
% theta = q4;
% d = 0;
% a = 10;
% alpha= 0;
% 
% T3_4 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
%     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
%     0 sin(alpha) cos(alpha) d
%     0 0 0 1];
% 
% T3_4 = vpa(T3_4,3)


T_04 = T0_1*T1_2*T2_3;
T_04 = simplify(T_04)
T_04 = vpa(T_04,4)

% For å sette inn verdiene
% T_04_ = subs(T_04,[q1 q2 q3 q4],[0 0 0 0]);
% T_04__ =double(T_04_)
