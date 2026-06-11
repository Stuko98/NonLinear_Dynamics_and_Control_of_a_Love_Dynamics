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
fig = figure;
hold on 
plot(out.logsout.get('x').Values.Data(:,1),out.logsout.get('x').Values.Data(:,2),'LineWidth',2) % (x1,x2)
plot(x1d,x2d,'LineWidth',2,'Marker','*')                                                      
hold off
axis padded
grid on
box on
ylabel('$x_2(t)$')
xlabel('$x_1(t)$')
legend('$x(t)$','$P_2$','FontSize',12,'Location','southeast'); 
%exportgraphics(fig, 'x_P2_phase_plane_iofbl.pdf');

% y, x2
fig = figure;
tiledlayout(2,1)
nexttile
hold on
plot(out.tout,out.logsout.get('x').Values.Data(:,1),'LineWidth',2)     % y = x1
plot(out.tout,yd*ones(size(out.tout)),'LineStyle','--','LineWidth',2) % yd
hold off
ylabel('$y(t), y_d$')
xlabel('$t$ [s]')
ylim padded
grid on
box on
legend('$y(t) = x_1(t)$','$y_d$','FontSize',12,'Location','southeast'); 
nexttile
plot(out.tout,out.logsout.get('x').Values.Data(:,2),'LineWidth',2)     % x2
ylabel('$x_2(t)$')
xlabel('$t$ [s]')
ylim padded
grid on
box on
%exportgraphics(fig, 'y_x2_iofbl.pdf');

% u
fig = figure;
plot(out.tout, out.logsout.get('u').Values.Data, 'LineWidth', 2); % u
ylabel('$u(t)$')
xlabel('$t$ [s]')
ylim padded
grid on
box on 
%exportgraphics(fig, 'u_iofbl.pdf');

% e
fig = figure;
plot(out.tout,out.logsout.get('e').Values.Data, 'LineWidth', 2); % e = yd - y
ylabel('$e(t)$')
xlabel('$t$ [s]')
ylim padded
grid on
box on 
%exportgraphics(fig, 'e_iofbl.pdf');

% d
fig = figure;
plot(out.tout,out.logsout.get('d').Values.Data,'LineWidth',2)
ylabel('$d(t)$')
xlabel('$t$ [s]')
ylim padded
grid on
box on
legend('$d_1(t)$','$d_2(t)$','FontSize',12,'Location','southeast'); 
%exportgraphics(fig, 'd.pdf');


%% ============================================================
%  Selected nonlinear SISO specifications from Simulink output
%  Signals:
%       y(t) = x1(t)
%       r(t) = yd
%       u(t) = control input
%       e(t) = yd - y(t)
% ============================================================

%% Extract signals

t = out.tout(:);

x = out.logsout.get('x').Values.Data;
y = x(:,1);              % output y = x1

r = yd * ones(size(t));  % constant reference

u = out.logsout.get('u').Values.Data(:);

e = r - y;

%% User-defined options
% tempo di assestamento al (100-epsilon)%
settlingTolerance = 0.05;     % == 2% of the final value of y == settling band for settling time ts. MODIFICA QUI
finalSamples = min(50, length(t));

%% Final values

r_final = mean(r(end-finalSamples+1:end));
y_final = mean(y(end-finalSamples+1:end));
u_final = mean(u(end-finalSamples+1:end));
e_final = mean(e(end-finalSamples+1:end));

%% ============================================================
%  Tracking error specifications
% ============================================================

steady_state_error = e_final;

absolute_steady_state_error = abs(steady_state_error);

maximum_absolute_error = max(abs(e));

%% ============================================================
%  Time-domain specifications
% ============================================================

y0 = y(1);
rf = r_final;

% Rise time 10%-90%
y10 = y0 + 0.1*(rf - y0);
y90 = y0 + 0.9*(rf - y0);

if rf >= y0
    idx10 = find(y >= y10, 1, 'first');
    idx90 = find(y >= y90, 1, 'first');
else
    idx10 = find(y <= y10, 1, 'first');
    idx90 = find(y <= y90, 1, 'first');
end

if ~isempty(idx10) && ~isempty(idx90)
    rise_time = t(idx90) - t(idx10);
    t10 = t(idx10);
    t90 = t(idx90);
else
    rise_time = NaN;
    t10 = NaN;
    t90 = NaN;
end

% Settling time
settling_band = settlingTolerance * max(abs(rf), 1e-9);

inside_settling_band = abs(y - rf) <= settling_band;

settling_time = NaN;

for k = 1:length(t)
    if all(inside_settling_band(k:end))
        settling_time = t(k);
        break
    end
end

% Maximum output value
maximum_output_value = max(y);

% Maximum overshoot percentage and peak time
if rf >= y0
    [peak_value, idx_peak] = max(y);

    if abs(rf) > 1e-9
        maximum_overshoot_percentage = ...
            max(0, (peak_value - rf)/abs(rf) * 100);
    else
        maximum_overshoot_percentage = NaN;
    end
else
    [peak_value, idx_peak] = min(y);

    if abs(rf) > 1e-9
        maximum_overshoot_percentage = ...
            max(0, (rf - peak_value)/abs(rf) * 100);
    else
        maximum_overshoot_percentage = NaN;
    end
end

peak_time = t(idx_peak);

%% ============================================================
%  Control effort specifications
% ============================================================

maximum_control_effort = max(abs(u));

control_minimum_value = min(u);

control_maximum_value = max(u);

%% ============================================================
%  Store selected specifications in a structure
% ============================================================

specs = struct();

% Final values
specs.r_final = r_final;
specs.y_final = y_final;
specs.u_final = u_final;
specs.e_final = e_final;

% Tracking error
specs.steady_state_error = steady_state_error;
specs.absolute_steady_state_error = absolute_steady_state_error;
specs.maximum_absolute_error = maximum_absolute_error;

% Time-domain
specs.rise_time = rise_time;
specs.t10 = t10;
specs.t90 = t90;
specs.settling_time = settling_time;
specs.settling_tolerance = settlingTolerance;
specs.settling_band = settling_band;
specs.maximum_output_value = maximum_output_value;
specs.maximum_overshoot_percentage = maximum_overshoot_percentage;
specs.peak_time = peak_time;
specs.peak_value = peak_value;

% Control
specs.maximum_control_effort = maximum_control_effort;
specs.control_minimum_value = control_minimum_value;
specs.control_maximum_value = control_maximum_value;

%% ============================================================
%  Print selected results
% ============================================================

fprintf('\n===========================================\n');
fprintf(' NONLINEAR SISO CONTROL SPECIFICATIONS\n');
fprintf('===========================================\n');

fprintf('\n--- Final values ---\n');
fprintf('Reference final value        r_f    : %.4f\n', r_final);
fprintf('Output final value           y_f    : %.4f\n', y_final);
fprintf('Control final value          u_f    : %.4f\n', u_final);

fprintf('\n--- Tracking error specifications ---\n');
fprintf('Steady-state error           e_ss   : %.4f\n', steady_state_error);
fprintf('Absolute steady-state error |e_ss|  : %.4f\n', absolute_steady_state_error);
fprintf('Maximum absolute error       e_max  : %.4f\n', maximum_absolute_error);

fprintf('\n--- Time-domain specifications ---\n');
fprintf('Rise time                    t_r    : %.4f s\n', rise_time);
fprintf('Settling time (%.1f%% band)    t_s    : %.4f s\n', settlingTolerance*100, settling_time);
fprintf('Maximum output value         y_max  : %.4f\n', maximum_output_value);
fprintf('Maximum overshoot percentage S%%     : %.4f %%\n', maximum_overshoot_percentage);
fprintf('Peak time                    t_p    : %.4f s\n', peak_time);
fprintf('Peak value                   y_p    : %.4f\n', peak_value);

fprintf('\n--- Control effort specifications ---\n');
fprintf('Maximum control effort       u_abs,max  : %.4f\n', maximum_control_effort);
fprintf('Control minimum value        u_min      : %.4f\n', control_minimum_value);
fprintf('Control maximum value        u_max      : %.4f\n', control_maximum_value);

fprintf('===========================================\n\n');