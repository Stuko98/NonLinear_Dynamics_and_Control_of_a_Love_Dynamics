clc
close all
clear all

%% nonlinear system
x = sym('x', [1 2], 'real'); % creo le variabili di stato x = (x1,x2) e le forzo ad essere reali
syms A1 A2 k real % parameters
n = length(x); % == 2, system dimension

% vector field
f = [-x(1) + A2 + k*x(2)*exp(-x(2));      % vector field f: R^n -> R^n
     -x(2) + A1 + k*x(1)*exp(-x(1))];          
g = sym([0 1])';                          % vector field g: R^n -> R^n

% Nel momento in cui le variabili di f (x = (x1,x2)) sono
% dichiarate symbolic, tutto f è automaticamente symbolic)

%% ISFBL conditions
if n == 1
    disp("n = 1 -> devo considerare solo g -> tutte le condizioni dell'ISFBL theorem sono automaticamente soddisfatte")

% condition 1: linear independence
elseif n > 1
    cond1 = sym(zeros(n,n));        
    cond1(:,1) = g;
    i = 2; % contatore delle colonne in cond1
    j = 1; % contatore del while alla riga 34
    while j <= n-1
        cond1(:,i) = rec_lie_br(f,g,x,j);     % [g adf(g) ... adf^(n-1)(g)]
        i = i+1;
        j = j+1; 
    end
end
det(cond1)~= 0;  % -k*exp(-x2)*(x2 - 1) ~= 0.      g e liebr linearly indipendent <-> x2 ~= 1 


% condition 2: involutivity
if n == 2
    disp("n = 2 -> dobbiamo considerare fino ad adfg^(n-2) = adfg^0 = g. Quindi considereremo solo g. E sappiamo che un unico vector field è sempre involutive!")

elseif n > 2
    cond2 = cond1(:,1:end-1);          % [g adf(g) ... adf^(n-2)(g)]
    cont = 0;                          % contatore per verificare quante delle colonne sono dei constant vector field 
    for j = 1:size(cond2,2)            % scorro colonne
        if isempty(symvar(cond2(:,j))) % stampa un vettore delle variabili della singola colonna di cond2. Verifica se la colonna è vuota di variabili, cioè se la colonna è fatta da sole costanti
            disp('almeno un vector field è constant -> condition 2 è soddisfatta!')
            break                      % se ho trovato almeno un constant vector field, esco dal loop
        end
        cont = cont + 1;
    end

    if cont == size(cond2,2)           % se nessuna delle colonne è un constant vector field...
        disp('Bisogna verificare involutivity in altro modo')
    end
end

    

%% T1 function 
syms T1(x1,x2)
eqn  = sym(zeros(1,n-1));       % initialization
eqn(1) = lie_der(T1,g,x) == 0;  % -> diff(T1(x1, x2), x2) == 0
for i = 2:n-1
    eqn(i) = rec_lie_der(T1,f,x,i-1);
    eqn(i) = lie_der(eqn(i),g,x);
    eqn(i) = eqn(i) == 0;
end

% dobbiamo usare una scalar function T1: R^n -> R tale che valgono le
% condizioni in eqn --> una scelta semplice è T1 = x1

T1 = x(1);

%% state transformation

z = sym('z', [1 n], 'real');         % forzo le variabili di stato z = (z1,z2,...,zn) ad essere reali

z(1) = T1;                           % scalar value \in R
for i = 1:n-1
    z(i+1) = rec_lie_der(T1,f,x,i);  % scalar value \in R
end


l1 = rec_lie_der(T1,f,x,n-1);
l1 = lie_der(l1,g,x);                % == Lg(Lf^n-1(T1))

l2 = rec_lie_der(T1,f,x,n);          % == Lf^n(T1)

l1 = simplify(l1);
l2 = simplify(l2);

% i risultati di z, l1 e l2 li copi e incolli  per il IS FBL nel Simulink
% -> u = (1/l1)*(-l2 + v)


%% MRAC-MCS controller
% nonlinear system  --IS FBL-->

% A = [0 1;
%      0 0];
% B = [0 1]';

% the nonlinear system turns into a  (Brunovsky) controllable canonical
% form -> with this condition, we can apply a MRAC-MCS controller!

% controller parameters
alfa = 10;
beta = 1;

% model reference
ts = 0.5; % s           % settling time
Am = [   0         1;
    -16/ts^2   -8/ts];
Bm = [0 16/ts^2]';

xm0 = [0 0]';

Q = [1 0;               % scelta standard
     0 1];
P = lyap(Am',Q);        % occhio alla funzione lyap(): serve che inserisci non Am, ma Am', perché risolve l'eq di lyapunov in forma trasposta!
Be = [0 1]';

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

%% simulation
out = sim("control_system_mrac_mcs.slx");
run("plots_mrac_mcs.m");

%% functions

% lie derivative
function ld = lie_der(sca_fun,vec_fie,x) 
    ld = gradient(sca_fun,x).'*vec_fie; % prodotto scalare. gradient è in forma colonna -> lo porto in forma riga. In più, non uso ' (che traspone non solo la matrice, ma anche i valori), ma .' (che traspone solo la matrice)
end



% recursive lie derivative
function rld = rec_lie_der(sca_fun,vec_fie,x,i)   

    % caso base
    if i == 1 
        rld = lie_der(sca_fun,vec_fie,x);

    % chiamata ricorsiva -> la funzione richiama sé stessa, finché non arriva al caso base
    else
        rld = lie_der(rec_lie_der(sca_fun,vec_fie,x,i-1),vec_fie,x);

    end

end



% lie brackets
function lb = lie_br(f,g,x)
    lb = jacobian(g,x)*f - jacobian(f,x)*g; % == adfg
end



% recursive lie brackets
function rlb = rec_lie_br(f,g,x,i)   

    % caso base
    if i == 1 
        rlb = lie_br(f,g,x);

    % chiamata ricorsiva -> la funzione richiama sé stessa, finché non arriva al caso base
    else
        rlb = lie_br(f,rec_lie_br(f,g,x,i-1),x);

    end

end






