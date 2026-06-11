close all
clear all
clc

%% nonlinear system
x = sym('x', [1 2], 'real'); % creo le variabili di stato x = (x1,x2) e le forzo ad essere reali

% parameters
A1 = 0;
A2 = 0;
k = 15;
 
% vector field
f1 = -x(1) + A2 + k*x(2)*exp(-x(2));
f2 = -x(2) + A1 + k*x(1)*exp(-x(1));
f = [f1 f2]';

% equilibria
P1 = vpasolve(f == 0, x, [1; 6]); % stable node 1
P2 = vpasolve(f == 0, x, [4; 4]); % saddle
P3 = vpasolve(f == 0, x, [6; 1]); % stable node 2

disp('stable node 1:');
disp(['(x1,x2) = (', num2str(double(P1.x1)),',' num2str(double(P1.x2)),')']);
disp(' ');
disp('saddle:');
disp(['(x1,x2) = (', num2str(double(P2.x1)),',' num2str(double(P2.x2)),')']);
disp(' ');
disp('stable node 2:');
disp(['(x1,x2) = (', num2str(double(P3.x1)),',' num2str(double(P3.x2)),')']);
disp(' ');

% jacobian
J = simplify(jacobian(f,x)); 

% initial conditions of the nonlinear system (for Simulink)
x0 = [1 1.5]';

%% linearized system (in the saddle point S2)

% Obiettivo: linearizzare il sistema in un punto di equilibrio a scelta, e
% controllare il sistema linearizzato. 
% In corrispondenza di un ingresso di equilibrio ueq = 0, ottengo che gli
% stati di equilibrio xeq sono gli stessi di prima. Inoltre, scelgo come
% stato di equilibrio S2, cioè la sella.
% Quindi il punto di equilibrio attorno a cui linearizzerò il sistema è
% (xeq,ueq) = (P2,0).

syms u real % introduco anche l'ingresso

% % vector field
% f1 = -x(1) + A2 + k*x(2)*exp(-x(2));
% f2 = -x(2) + A1 + k*x(1)*exp(-x(1)) + u;
% f = [f1 f2]';
% 
% % Jacobian
% J = simplify(jacobian(f,x)); % jacobiano --> è lo stesso del sistema a ciclo aperto

ueq = 0; % considero un ingresso di equilibrio nullo. In corrispondenza di esso, il sistema linearizzato sarà lo stesso visto a ciclo aperto
xeq = P2; % scelgo di linearizzare attorno al punto di eq sella
J_xeq_ueq = subs(J,[x(1) x(2) u],[xeq.x1 xeq.x2 ueq]); % jacobiano linearizzato nel punto di equilibrio S2 (sella) 
[v, lambda]= eig(J_xeq_ueq); % autovalori (lambda(1,1) e lambda(2,2)) e autovettori (v(:,1) e v(:,2)) si trovano con pplane8! (verificato)

xeq.x1 = double(xeq.x1); % sym  --> double, in modo da poterlo usare dopo su Simulink
xeq.x2 = double(xeq.x2); % sym  --> double, in modo da poterlo usare dopo su Simulink
yd = xeq.x1; % reference

A = double(J_xeq_ueq); % sym  --> double, in modo da poterlo usare dopo su Simulink
B = [0 1]'; % applico il controllo all'amore di Rhett
C = [1 0]; % applicherò un controllore con anche l'azione integrale -> prelevo SOLO UN'USCITA!!! Serve per augmented system
D = 0; % D cambia di conseguenza 

ltisys = ss(A,B,C,D); % state space lti system. Plottalo  su pplane8.m
G = tf(ltisys); % conversion state space --> transfer function matrix G(s)


%% state feedback controller (SFC) for the linearized system (reference tracking)

% controllability
rank(ctrb(A,B)); % == 2 --> the system linearized in the S2 point is controllable 

% poles assignment based controller + integral action for reference tracking
% Nel caso di reference tracking, dovrò considerare l'augmented system 
A_aug = [A   zeros(2,1);
        -C       0];                         % farò il tracking di UNA uscita -> aggiungo UNA colonna di zeri alla fine della matrice
B_aug = [B;0];                               % farò il tracking di UNA uscita -> aggiungo UNO zero alla colonna di B
K_aug = - place(A_aug,B_aug,[-1 -1.1 -1.2]); % place implementa un metodo numerico molto utile nei sistemi MIMO per assegnare i valori di K in modo che gli autovalori di A+B*K siano quelli specificati nel terzo input. Gli autovalori specificati nel terzo input non hanno seguito un criterio, ma sono semplicemente stati posti nella parte sinistra del piano Re-Im
K = K_aug(1:end-1);                          % controller gain matrix
ki = K_aug(end);                             % integral action gain
eig(A_aug + B_aug*K_aug);                    % check -> ok



%% closed Loop lti system
% A_cl = A_aug + B_aug*K_aug;
% B_cl = [0 1 0]'; % in realtà il closed-loop system è autonomo quindi B_cl = [0 0], ma mi serve mantenere l'ingresso originale per comprendere quali sono gli effetti degli ingressi sulle uscite
% C_cl = eye(2,3);
% D_cl = [0 0]';
% 
% ltisys_cl = ss(A_cl,B_cl,C_cl,D_cl); % Plottalo  su pplane8.m
% G_cl = tf(ltisys_cl);

%% simulation
out = sim("control_system_nl_lin_ref.slx");
run("plots_nl_lin_ref.m");



