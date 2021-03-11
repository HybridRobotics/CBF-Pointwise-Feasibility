close all; clear all; clc;
%% ACC parameters
param = AdaptiveCruiseControlParam();

%% Create different initial conditions
[xaxis_grid, yaxis_grid, zaxis_grid] = meshgrid(0, 26:2:32, 100);
x0_list = [xaxis_grid, yaxis_grid, zaxis_grid]';

%% Simulate system with different control policies
data_acc_system_clf_cbf_qp = {};
data_acc_system_soft_decay_clf_cbf_qp = {};
for i = 1:size(x0_list, 2)
    % simulate system with CLF-CBF-QP
    fprintf('simulate system under CLF-CBF-QP with v0 as %f\n', x0_list(2,i));
    acc_system = AdaptiveCruiseControl(x0_list(:,i));
    acc_system.method_flag = false;
    acc_system.dt = 0.01;
    acc_system.sim(12);
    data_acc_system_clf_cbf_qp{i} = acc_system;
    % simulate system with Soft-decay CLF-CBF-QP
    fprintf('simulate system under Soft-decay CLF-CBF-QP with v0 as %f\n', x0_list(2,i));
    acc_system = AdaptiveCruiseControl(x0_list(:,i));
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
time1 = data_acc_system_clf_cbf_qp{3}.time_log(end);
time2 = data_acc_system_clf_cbf_qp{4}.time_log(end);

%% velocity plotting
figure('Renderer', 'painters', 'Position', [0 0 600 600]);
set(gca,'LooseInset',get(gca,'TightInset'));
hold on
% grid on
for i = 1:size(x0_list, 2)
    acc_system = data_acc_system_clf_cbf_qp{i};
    plot(acc_system.time_log, acc_system.x_log(2,:), '-.', 'Color', color_band(i,:), 'LineWidth', 8.0);
    acc_system = data_acc_system_soft_decay_clf_cbf_qp{i};
    plot(acc_system.time_log, acc_system.x_log(2,:), 'Color', color_band(i,:), 'LineWidth', 3.0);
end
plot([time1, time1], [16,32], '--', 'Color', color_band(3,:), 'LineWidth', 1.5);
plot([time2, time2], [16,32], '--', 'Color', color_band(4,:), 'LineWidth', 1.5);
ax = gca;
ax.XTick = [0:3:12];
ax.YTick = [16:2:32];
set(gca,'LineWidth', 0.2, 'FontSize', 25);
xlabel('Time (s)','interpreter','latex','FontSize', 25);
ylabel('$x_2(t)$ (m/s)','interpreter','latex','FontSize', 25);
print(gcf,'figures/feasibility/benchmark-feasibility-velocity.png', '-dpng', '-r1000');
print(gcf,'figures/feasibility/benchmark-feasibility-velocity.eps', '-depsc');

%% control input plotting
figure('Renderer', 'painters', 'Position', [0 0 600 600]);
set(gca,'LooseInset',get(gca,'TightInset'));
hold on
% grid on
for i = 1:size(x0_list, 2)
    acc_system = data_acc_system_clf_cbf_qp{i};
    plot(acc_system.time_log, acc_system.u_log/1000, '--', 'Color', color_band(i,:), 'LineWidth', 8.0);
    acc_system = data_acc_system_soft_decay_clf_cbf_qp{i};
    plot(acc_system.time_log, acc_system.u_log/1000, 'Color', color_band(i,:), 'LineWidth', 3.0);
end
% plot input lower bound
plot(acc_system.time_log, -param.c_a * param.m * param.g .* ones(size(acc_system.time_log,2))/1000, 'k--', 'LineWidth', 1.5);
plot([time1, time1], [-5, 4], '--', 'Color', color_band(3,:), 'LineWidth', 1.5);
plot([time2, time2], [-5, 4], '--', 'Color', color_band(4,:), 'LineWidth', 1.5);
ax = gca;
ax.XTick = [0:3:12];
ax.YTick = [-5:1:5];
set(gca,'LineWidth', 0.2, 'FontSize', 25);
xlabel('Time (s)','interpreter','latex','FontSize', 25);
ylabel('$\mathbf{u}(t)$ (kN)','interpreter','latex','FontSize', 25);
print(gcf,'figures/feasibility/benchmark-feasibility-control-input.png', '-dpng', '-r1000');
print(gcf,'figures/feasibility/benchmark-feasibility-control-input.eps', '-depsc', '-opengl', '-r600');

%% cbf plotting
figure('Renderer', 'painters', 'Position', [0 0 600 600]);
set(gca,'LooseInset',get(gca,'TightInset'));
hold on
% grid on
for i = 1:size(x0_list, 2)
    acc_system = data_acc_system_clf_cbf_qp{i};
    plot(acc_system.time_log, acc_system.cbf_log, '--', 'Color', color_band(i,:), 'LineWidth', 8.0);
    acc_system = data_acc_system_soft_decay_clf_cbf_qp{i};
    plot(acc_system.time_log, acc_system.cbf_log, 'Color', color_band(i,:), 'LineWidth', 3.0);
end
plot([time1, time1], [0,60], '--', 'Color', color_band(3,:), 'LineWidth', 1.5);
plot([time2, time2], [0,60], '--', 'Color', color_band(4,:), 'LineWidth', 1.5);
ax = gca;
ax.XTick = [0:3:12];
ax.YTick = [0:20:60];
set(gca,'LineWidth', 0.2, 'FontSize', 25);
xlabel('Time (s)','interpreter','latex','FontSize', 25);
ylabel('$h(t)$','interpreter','latex','FontSize', 25);
h=get(gca,'Children');
h_legend = legend(h([end-2:end-9]),...
    {'Nominal $v(0)=26$m/s', 'Optimal-decay $v(0)=26$m/s',...
    'Nominal $v(0)=28$m/s','Optimal-decay $v(0)=28$m/s',...
    'Nominal $v(0)=30$m/s', 'Optimal-decay $v(0)=30$m/s',...
    'Nominal $v(0)=32$m/s', 'Optimal-decay $v(0)=32$m/s'}, 'Location', 'NorthEast');
h_legend.FontSize = 15;
set(h_legend, 'Interpreter','latex');
print(gcf,'figures/feasibility/benchmark-feasibility-cbf.png', '-dpng', '-r1000');
print(gcf,'figures/feasibility/benchmark-feasibility-cbf.eps', '-depsc');

%% omega plotting
figure('Renderer', 'painters', 'Position', [0 0 600 600]);
set(gca,'LooseInset',get(gca,'TightInset'));
hold on
% grid on
for i = 1:size(x0_list, 2)
    acc_system = data_acc_system_soft_decay_clf_cbf_qp{i};
    plot(acc_system.time_log, acc_system.omega_log, 'Color', color_band(i,:), 'LineWidth', 3.0);
end
ax = gca;
ax.XTick = [0:3:12];
ax.YTick = [1:0.3:1.5];
set(gca,'LineWidth', 0.2, 'FontSize', 25);
xlabel('Time (s)','interpreter','latex','FontSize', 25);
ylabel('$\omega(t)$','interpreter','latex','FontSize', 25);
print(gcf,'figures/feasibility/benchmark-feasibility-omega.png', '-dpng', '-r1000');
print(gcf,'figures/feasibility/benchmark-feasibility-omega.eps', '-depsc');
