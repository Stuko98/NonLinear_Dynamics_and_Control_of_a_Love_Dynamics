function out = chemical_reactor_DCNL
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
function dydt = fun_eval(t,kmrgd,par_mumu,par_k_bar)
dydt=[par_mumu-par_k_bar*kmrgd(1)-kmrgd(1)*kmrgd(2)^2;;
par_k_bar*kmrgd(1)+kmrgd(1)*kmrgd(2)^2-kmrgd(2);;];

% --------------------------------------------------------------------------
function [tspan,y0,options] = init
handles = feval(chemical_reactor_DCNL);
y0=[0,0];
options = odeset('Jacobian',handles(3),'JacobianP',handles(4),'Hessians',handles(5),'HessiansP',handles(6));
tspan = [0 10];

% --------------------------------------------------------------------------
function jac = jacobian(t,kmrgd,par_mumu,par_k_bar)
jac=[ - par_k_bar - kmrgd(2)^2 , -2*kmrgd(1)*kmrgd(2) ; par_k_bar + kmrgd(2)^2 , 2*kmrgd(1)*kmrgd(2) - 1 ];
% --------------------------------------------------------------------------
function jacp = jacobianp(t,kmrgd,par_mumu,par_k_bar)
jacp=[ 1 , -kmrgd(1) ; 0 , kmrgd(1) ];
% --------------------------------------------------------------------------
function hess = hessians(t,kmrgd,par_mumu,par_k_bar)
hess1=[ 0 , -2*kmrgd(2) ; 0 , 2*kmrgd(2) ];
hess2=[ -2*kmrgd(2) , -2*kmrgd(1) ; 2*kmrgd(2) , 2*kmrgd(1) ];
hess(:,:,1) =hess1;
hess(:,:,2) =hess2;
% --------------------------------------------------------------------------
function hessp = hessiansp(t,kmrgd,par_mumu,par_k_bar)
hessp1=[ 0 , 0 ; 0 , 0 ];
hessp2=[ -1 , 0 ; 1 , 0 ];
hessp(:,:,1) =hessp1;
hessp(:,:,2) =hessp2;
%---------------------------------------------------------------------------
function tens3  = der3(t,kmrgd,par_mumu,par_k_bar)
tens31=[ 0 , 0 ; 0 , 0 ];
tens32=[ 0 , -2 ; 0 , 2 ];
tens33=[ 0 , -2 ; 0 , 2 ];
tens34=[ -2 , 0 ; 2 , 0 ];
tens3(:,:,1,1) =tens31;
tens3(:,:,1,2) =tens32;
tens3(:,:,2,1) =tens33;
tens3(:,:,2,2) =tens34;
%---------------------------------------------------------------------------
function tens4  = der4(t,kmrgd,par_mumu,par_k_bar)
%---------------------------------------------------------------------------
function tens5  = der5(t,kmrgd,par_mumu,par_k_bar)
