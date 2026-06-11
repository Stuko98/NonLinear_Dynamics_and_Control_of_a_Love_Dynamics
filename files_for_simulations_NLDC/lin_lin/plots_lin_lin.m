%% plot settings
% modifico a monte le impostazioni dei plots
set(groot, 'DefaultAxesFontSize', 12)                      % per i tick (numeri)
set(groot, 'DefaultAxesLabelFontSizeMultiplier', 1.2)      % multiplier che esprime quante volte xlabel e ylabel hanno unFontSize maggiore dei tick
set(groot, 'DefaultTextInterpreter', 'latex')              % per tutto tranne sgtitle
set(groot, 'DefaultAxesTickLabelInterpreter', 'latex')

set(groot, 'defaultLegendInterpreter', 'latex');        
%set(groot, 'defaultLegendOrientation', 'horizontal'); 
%set(groot, 'defaultLegendLocation', 'southoutside');

%% plots
% x vs P2 in the phase plane
f = figure;
hold on 
plot(out.logsout.get('y = x').Values.Data(:,1),out.logsout.get('y = x').Values.Data(:,2),'LineWidth',2) % (x1,x2)
plot(xeq.x1,xeq.x2,'LineWidth',2,'Marker','*')                                                      
hold off
axis padded
grid on
box on
ylabel('$x_2(t)$')
xlabel('$x_1(t)$')
legend('$x(t)$','$P_2$','FontSize',12,'Location','southeast'); 
exportgraphics(f, 'x_P2_phase_plane_lin_lin.pdf');

% y, x2
f = figure;
tiledlayout(2,1)
nexttile
hold on
plot(out.tout,out.logsout.get('y = x').Values.Data(:,1),'LineWidth',2)     % x1
plot(out.tout,xeq.x1*ones(size(out.tout)),'LineStyle','--','LineWidth',2)  % x1d
hold off
ylabel('$y(t), y_d$')
xlabel('$t$ [s]')
ylim padded
grid on
box on
legend('$y(t) = x_1(t)$','$y_d$','FontSize',12,'Location','southeast'); 
nexttile
plot(out.tout,out.logsout.get('y = x').Values.Data(:,2),'LineWidth',2)     % x2
ylabel('$x_2(t)$')
xlabel('$t$ [s]')
ylim padded
grid on
box on
exportgraphics(f, 'y_x2_lin_lin.pdf');

% u
f = figure;
plot(out.tout, out.logsout.get('u').Values.Data, 'LineWidth', 2); % u
ylabel('$u(t)$')
xlabel('$t$ [s]')
ylim padded
grid on
box on 
exportgraphics(f, 'u_lin_lin.pdf');

% e
f = figure;
hold on
plot(out.tout,out.logsout.get('e').Values.Data(:,1), 'LineWidth', 2); % e = y - yd
plot(out.tout,zeros(size(out.tout)),'LineStyle','--','LineWidth',2);
hold off
ylabel('$e(t)$')
xlabel('$t$ [s]')
ylim padded
grid on
box on 
exportgraphics(f, 'e_lin_lin.pdf');