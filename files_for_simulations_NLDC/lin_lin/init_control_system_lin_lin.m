close all
clear all
clc

%% nonlinear system
x = sym('x', [1 2], 'real'); 

% parameters
A1 = 0;
A2 = 0;
k = 15;
 
% vector field
f1 = -x(1) + A2 + k*x(2)*exp(-x(2));
f2 = -x(2) + A1 + k*x(1)*exp(-x(1));
f = [f1 f2]';

% equilibria
S1 = vpasolve(f == 0, x, [1; 6]); % stable node 1
S2 = vpasolve(f == 0, x, [4; 4]); % saddle
S3 = vpasolve(f == 0, x, [6; 1]); % stable node 2

disp('stable node 1:');
disp(['(x1,x2) = (', num2str(double(S1.x1)),',' num2str(double(S1.x2)),')']);
disp(' ');
disp('saddle:');
disp(['(x1,x2) = (', num2str(double(S2.x1)),',' num2str(double(S2.x2)),')']);
disp(' ');
disp('stable node 2:');
disp(['(x1,x2) = (', num2str(double(S3.x1)),',' num2str(double(S3.x2)),')']);
disp(' ');

% jacobian
J = simplify(jacobian(f,x));


%% linearized system (in the saddle point S2)

% Obiettivo: linearizzare il sistema in un punto di equilibrio a scelta, e
% controllare il sistema linearizzato. 
% In corrispondenza di un ingresso di equilibrio ueq = 0, ottengo che gli
% stati di equilibrio xeq sono gli stessi di prima. 
% Scelgo come stato di equilibrio S2, cioè la sella ->
% il punto di equilibrio attorno a cui linearizzerò il sistema è 
% (xeq,ueq) = (S2,0).

syms u real % introduco anche l'ingresso

% % vector field
% f1 = -x(1) + A2 + k*x(2)*exp(-x(2));
% f2 = -x(2) + A1 + k*x(1)*exp(-x(1)) + u;
% f = [f1 f2]';
% 
% % jacobian
% J = simplify(jacobian(f,x)); % jacobiano --> è lo stesso del sistema a ciclo aperto

ueq = 0; % considero un ingresso di equilibrio nullo. In corrispondenza di esso, il sistema linearizzato sarà lo stesso visto a ciclo aperto
xeq = S2; % scelgo di linearizzare attorno al punto di eq sella
J_xeq_ueq = subs(J,[x(1) x(2) u],[xeq.x1 xeq.x2 ueq]); % jacobiano linearizzato nel punto di equilibrio S2 (sella) 
[v, lambda]= eig(J_xeq_ueq); % autovalori (lambda(1,1) e lambda(2,2)) e autovettori (v(:,1) e v(:,2)) si trovano con pplane8! (verificato)

xeq.x1 = double(xeq.x1); % sym  --> double, in modo da poterlo usare dopo su Simulink
xeq.x2 = double(xeq.x2); % sym  --> double, in modo da poterlo usare dopo su Simulink

A = double(J_xeq_ueq); % sym  --> double, in modo da poterlo usare dopo su Simulink
B = [0 1]'; % applico il controllo all'amore di Rhett
C = eye(2); % quando farò la conversion state space --> transfer function G(s), mi servirà così per capire la transfer function da u sia verso y1 che y2
D = [0 0]';

ltisys = ss(A,B,C,D); % state space lti system. Plottalo  su pplane8.m
G = tf(ltisys); % conversion state space --> transfer function matrix G(s)


%% state feedback controller (SFC) for the linearized system (stabilization)

% controllability
rank(ctrb(A,B)); % == 2 --> the system linearized in the equilibrium xeq is controllable --> I can apply a poles assignment based controller

% poles assignment based controller
K = -place(A,B,[-1 -1.1]); % gain matrix
eig(A + B*K);              % eigenvalues of the closed loop system --> corrispondono a quanto voluto con place() --> ok


%% closed loop lti system
A_cl = A + B*K;
B_cl = [0 1]'; % in realtà il closed-loop system è autonomo quindi B_cl = [0 0], ma mi serve mantenere l'ingresso originale per comprendere quali sono gli effetti degli ingressi sulle uscite
C_cl = eye(2);
D_cl = [0 0]';

ltisys_cl = ss(A_cl,B_cl,C_cl,D_cl); % state space lti system. Plottalo  su pplane8.m
G_cl = tf(ltisys_cl); % conversion state space --> transfer function matrix G(s). Da qui capisco gli andamenti di y1(t) e y2(t) che vedrò su Simulink


% initial conditions of the nonlinear system (for Simulink)
x0 = [1 1.5]';

%% simulation
out = sim("control_system_lin_lin.slx");
run("plots_lin_lin.m");