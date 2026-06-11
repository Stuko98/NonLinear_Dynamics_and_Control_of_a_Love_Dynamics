clc
close all
clear all

%% nonlinear system
x = sym('x', [1 2], 'real');
syms A1 A2 k p1 p2 x1d x2d real

% vector field
f = [-x(1) + A2 + k*x(2)*exp(-x(2));      % vector field f: R^n -> R^n
     -x(2) + A1 + k*x(1)*exp(-x(1))];           
g = sym([0 1])';                          % vector field g: R^n -> R^n

%  OCCHIO: spesso g ha la forma di vettore di costanti, quindi viene
%  interpretato da matlab come numerical vector. Il problema è che dovremo
%  usare "jacobian()", che lavora solamente con symbolic vectors. Quindi
%  nel dubbio converto g in symbolic!

% OCCHIO: dovrei farlo anche con f, ma è improbabile che f sia un vettore
% di sole costanti. Nel momento in cui le sue variabili (x = x1,x2) sono
% dichiarate symbolic, tutto f è automaticamente symbolic)

%% sliding mode controller
sigma = p1*(x(1) - x1d) + p2*(x(2) - x2d);     % sigma(x). Scelto secondo trial and error

Lgsigma = lie_der(sigma,g,x);                  % == grad(sigma)*g == Lg(sigma) == p2 ~=0 (transversality condition check) scalar value \in R
Lfsigma = lie_der(sigma,f,x);                  % == grad(sigma)*f == Lf(sigma)     scalar value \in R

k_law = 1;                                     % control law gain: settandolo, definisco quanto intensamente la control law spinge le traiettorie verso lo switching manifold Sigma
u = 1/Lgsigma*(-Lfsigma - k_law*sign(sigma));  % control law

%% sliding vector field 
ueq = -Lfsigma/Lgsigma; % equivalent control input

Fs = simplify(f + g*ueq); % sliding vector field

eqn = solve(sigma,x(2));
sub = subs(Fs(1,:),x(2),eqn); 

Fs_on_Sigma = [sub;
               Fs(2:end,:)];                    % reduced order dynamics on the sliding manifold

Fs_on_Sigma = subs(Fs_on_Sigma,[p1 p2],[1 -1]); % reduced order dynamics on the sliding manifold + coefficient choice


%% for simulink
% parameters
A1 = 0;
A2 = 0;
k = 15;

x1d = 2.7081; % reference
yd = x1d;
x2d = 2.7081;

% initial conditions of the nonlinear system (for Simulink)
x0 = [1 1.5]';

% sliding vector field parameters: settandoli a mio piacimento, definisco
% come si comporta Fs, quindi la velocità di convergenza al reference:
% infatti il settling time ts ~ 4.6*p2/p1.
% Imponendo p1*p2<0, si ottiene il comportamento desiderato
p1 = 1;
p2 = -1;


% i risultati di cui sopra di 
% - sigma
% - Lgsigma
% - Lfsigma 
% li copi e incolli  per il SMC in "untitled.slx"

%% simulation
out = sim("control_system_smc.slx");
run("plots_smc.m");

%% functions

% lie derivative
function ld = lie_der(sca_fun,vec_fie,x) 
    ld = gradient(sca_fun,x).'*vec_fie; % prodotto scalare. gradient è in forma colonna -> lo porto in forma riga. In più, non uso ' (che traspone non solo la matrice, ma anche i valori), ma .' (che traspone solo la matrice)
end
