clc
close all
clear all

%% nonlinear system
x = sym('x', [1 2], 'real'); 
syms A1 A2 k real % parameters
n = length(x); % == 2, system dimension
 
% vector field
f = [-x(1) + A2 + k*x(2)*exp(-x(2));      % vector field f: R^n -> R^n
     -x(2) + A1 + k*x(1)*exp(-x(1))];          
g = [0 1]';                               % vector field g: R^n -> R^n
h = x(1);                                 % output scalar function h: R^n -> R 

% Nel momento in cui le variabili di f (x = x1,x2) sono
% dichiarate symbolic, tutto f è automaticamente symbolic)

%% IOFBL
l1 = lie_der(h,g,x); % == grad(h)*g == Lg(h)        scalar value \in R

if l1 ~= 0
    l2 = lie_der(h,f,x);

else
    i = 0;
    while(l1 == 0)
        i = i+1;
        l1 = rec_lie_der(h,f,x,i);
        l1 = lie_der(l1,g,x);         % == Lg(Lf^r-1(h))
    end
    
    l2 = rec_lie_der(h,f,x,i+1);      % == Lf^r(h)
end

l1 = simplify(l1);
l2 = simplify(l2);

% i risultati di l1 e l2 li copi e incolli  per il IO FBL nel Simulink
% -> u = (1/l1)*(-l2 + v)

r = i + 1; % relative degree == 2, defined <-> -k*exp(-x2)*(x2 - 1) ~= 0 
if r == n
    disp("no internal dynamics" )
elseif r < n
    disp("exists internal dynamics")
else
    disp("problem")
end

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
out = sim("control_system_iofbl.slx");
run("plots_iofbl.m");

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