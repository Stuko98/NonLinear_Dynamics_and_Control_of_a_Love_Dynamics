function out = gone_with_the_wind
out{1} = @init;
out{2} = @fun_eval;
out{3} = @jacobian;
out{4} = @jacobianp;
out{5} = @hessians;
out{6} = @hessiansp;
out{7} = @der3;
out{8} = [];
out{9} = [];

% --------------------------------------------------------------------------
function dydt = fun_eval(t,kmrgd,par_A1,par_A2,par_k)
dydt=[-kmrgd(1)+par_k*kmrgd(2)*exp(-kmrgd(2));
-kmrgd(2)+par_k*kmrgd(1)*exp(-kmrgd(1));];

% --------------------------------------------------------------------------
function [tspan,y0,options] = init
handles = feval(gone_with_the_wind);
y0=[0,0];
options = odeset('Jacobian',handles(3),'JacobianP',handles(4),'Hessians',handles(5),'HessiansP',handles(6));
tspan = [0 10];

% --------------------------------------------------------------------------
function jac = jacobian(t,kmrgd,par_A1,par_A2,par_k)
jac=[ -1 , par_k*exp(-kmrgd(2)) - kmrgd(2)*par_k*exp(-kmrgd(2)) ; par_k*exp(-kmrgd(1)) - kmrgd(1)*par_k*exp(-kmrgd(1)) , -1 ];
% --------------------------------------------------------------------------
function jacp = jacobianp(t,kmrgd,par_A1,par_A2,par_k)
jacp=[ 0 , 0 , kmrgd(2)*exp(-kmrgd(2)) ; 0 , 0 , kmrgd(1)*exp(-kmrgd(1)) ];
% --------------------------------------------------------------------------
function hess = hessians(t,kmrgd,par_A1,par_A2,par_k)
hess1=[ 0 , 0 ; kmrgd(1)*par_k*exp(-kmrgd(1)) - 2*par_k*exp(-kmrgd(1)) , 0 ];
hess2=[ 0 , kmrgd(2)*par_k*exp(-kmrgd(2)) - 2*par_k*exp(-kmrgd(2)) ; 0 , 0 ];
hess(:,:,1) =hess1;
hess(:,:,2) =hess2;
% --------------------------------------------------------------------------
function hessp = hessiansp(t,kmrgd,par_A1,par_A2,par_k)
hessp1=[ 0 , 0 ; 0 , 0 ];
hessp2=[ 0 , 0 ; 0 , 0 ];
hessp3=[ 0 , exp(-kmrgd(2)) - kmrgd(2)*exp(-kmrgd(2)) ; exp(-kmrgd(1)) - kmrgd(1)*exp(-kmrgd(1)) , 0 ];
hessp(:,:,1) =hessp1;
hessp(:,:,2) =hessp2;
hessp(:,:,3) =hessp3;
%---------------------------------------------------------------------------
function tens3  = der3(t,kmrgd,par_A1,par_A2,par_k)
tens31=[ 0 , 0 ; 3*par_k*exp(-kmrgd(1)) - kmrgd(1)*par_k*exp(-kmrgd(1)) , 0 ];
tens32=[ 0 , 0 ; 0 , 0 ];
tens33=[ 0 , 0 ; 0 , 0 ];
tens34=[ 0 , 3*par_k*exp(-kmrgd(2)) - kmrgd(2)*par_k*exp(-kmrgd(2)) ; 0 , 0 ];
tens3(:,:,1,1) =tens31;
tens3(:,:,1,2) =tens32;
tens3(:,:,2,1) =tens33;
tens3(:,:,2,2) =tens34;
%---------------------------------------------------------------------------
function tens4  = der4(t,kmrgd,par_A1,par_A2,par_k)
%---------------------------------------------------------------------------
function tens5  = der5(t,kmrgd,par_A1,par_A2,par_k)
