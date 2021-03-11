close all; clear all; clc;
%% ACC parameters
param = AdaptiveCruiseControlParam();

%% Create different initial conditions
hyperparameter_list = [
    [1.0, 10^4];...
    [1.0, 10^8];...
    [0.5, 10^4];...
    [0.5, 10^8];
    ];

%% Simulate system with different control policies
data_acc_system_soft_decay_clf_cbf_qp = {};
for i = 1:size(hyperparameter_list, 1)
    % simulate system with Soft-decay CLF-CBF-QP
    acc_system = AdaptiveCruiseControl([0; 32; 100]);
    % different hyperparameters
    new_param = param; % deep copy
    new_param.omega0 = hyperparameter_list(i,1);
    new_param.p_sb = hyperparameter_list(i,2);
    fprintf('simulate system under Soft-decay CLF-CBF-QP with omega0 as %f and pomega as %f\n', [new_param.omega0, new_param.p_sb]);
    acc_system.param = new_param;
    acc_system.method_flag = true;
    acc_system.dt = 0.01;
    acc_system.sim(12);
    data_acc_system_soft_decay_clf_cbf_qp{i} = acc_system;
end

%% Compare performance
color_band = [[0, 0.4470, 0.7410]
    [0.8500, 0.3250, 0.0980];
    [0.9290, 0.6940, 0.1250];
    [0.4940, 0.1840, 0.5560];
    [0.4660, 0.6740, 0.1880];
    [0.6350, 0.0780, 0.1840];
    ];

%% velocity plotting
figure('Renderer', 'painters', 'Position', [0 0 600 600]);
set(gca,'LooseInset',get(gca,'TightInset'));
hold on
grid on
for i = 1:size(hyperparameter_list, 1)
    acc_system = data_acc_system_soft_decay_clf_cbf_qp{i};
    plot(acc_system.time_log, acc_system.x_log(2,:), 'Color', color_band(i,:), 'LineWidth', 3.0);
end
ax = gca;
ax.XTick = [0:3:15];
ax.YTick = [16:2:32];
set(gca,'LineWidth', 0.2, 'FontSize', 25);
xlabel('Time (s)','interpreter','latex','FontSize', 25);
ylabel('$x_2(t)$ (m/s)','interpreter','latex','FontSize', 25);
print(gcf,'figures/hyperparameter/benchmark-hyperparameter-velocity.png', '-dpng', '-r1000');
print(gcf,'figures/hyperparameter/benchmark-hyperparameter-velocity.eps', '-depsc');

%% control input plotting
figure('Renderer', 'painters', 'Position', [0 0 600 600]);
set(gca,'LooseInset',get(gca,'TightInset'));
hold on
grid on
for i = 1:size(hyperparameter_list, 1)
    acc_system = data_acc_system_soft_decay_clf_cbf_qp{i};
    plot(acc_system.time_log, acc_system.u_log/1e3, 'Color', color_band(i,:), 'LineWidth', 3.0);
end
% plot input lower bound
plot(acc_system.time_log, -param.c_a * param.m * param.g .* ones(size(acc_system.time_log,2))/1e3, 'k--', 'LineWidth', 1.5);
ax = gca;
ax.XTick = [0:3:15];
ax.YTick = [-5:1:5];
set(gca,'LineWidth', 0.2, 'FontSize', 25);
xlabel('Time (s)','interpreter','latex','FontSize', 25);
ylabel('$\mathbf{u}(t)$ (kN)','interpreter','latex','FontSize', 25);
print(gcf,'figures/hyperparameter/benchmark-hyperparameter-control-input.png', '-dpng', '-r1000');
print(gcf,'figures/hyperparameter/benchmark-hyperparameter-control-input.eps', '-depsc', '-opengl', '-r600');

%% cbf plotting
figure('Renderer', 'painters', 'Position', [0 0 600 600]);
set(gca,'LooseInset',get(gca,'TightInset'));
hold on
grid on
for i = 1:size(hyperparameter_list, 1)
    acc_system = data_acc_system_soft_decay_clf_cbf_qp{i};
    plot(acc_system.time_log, acc_system.cbf_log, 'Color', color_band(i,:), 'LineWidth', 3.0);
end
ax = gca;
ax.XTick = [0:3:15];
% ax.YTick = [0:20:60];
ax.GridAlpha = 0.5;
ax.GridLineStyle = '--';
set(gca, 'YScale', 'log');
set(gca,'LineWidth', 0.2, 'FontSize', 25);
xlabel('Time (s)','interpreter','latex','FontSize', 25);
ylabel('$h(t)$','interpreter','latex','FontSize', 25);
h=get(gca,'Children');
h_legend = legend(h([end-3, end-2, end-1, end]),...
    {'$\omega_0=1.0, p_{\omega} = 10^4$',...
    '$\omega_0=1.0, p_{\omega} = 10^8$',...
    '$\omega_0=0.5, p_{\omega} = 10^4$',...
    '$\omega_0=0.5, p_{\omega} = 10^8$'}, 'Location', 'NorthEast');
h_legend.FontSize = 20;
set(h_legend, 'Interpreter','latex');
print(gcf,'figures/hyperparameter/benchmark-hyperparameter-cbf.png', '-dpng', '-r1000');
print(gcf,'figures/hyperparameter/benchmark-hyperparameter-cbf.eps', '-depsc');

%% omega plotting
figure('Renderer', 'painters', 'Position', [0 0 600 600]);
set(gca,'LooseInset',get(gca,'TightInset'));
hold on
grid on
for i = 1:size(hyperparameter_list, 1)
    acc_system = data_acc_system_soft_decay_clf_cbf_qp{i};
    plot(acc_system.time_log, acc_system.omega_log, 'Color', color_band(i,:), 'LineWidth', 3.0);
end
ax = gca;
ax.XTick = [0:3:15];
% ax.YTick = [1:0.3:1.5];
set(gca,'LineWidth', 0.2, 'FontSize', 25);
xlabel('Time (s)','interpreter','latex','FontSize', 25);
ylabel('$\omega(t)$','interpreter','latex','FontSize', 25);
print(gcf,'figures/hyperparameter/benchmark-hyperparameter-omega.png', '-dpng', '-r1000');
print(gcf,'figures/hyperparameter/benchmark-hyperparameter-omega.eps', '-depsc');
