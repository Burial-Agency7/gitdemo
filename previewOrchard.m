%% 简化果园环境预览
% 快速预览三种不同复杂度的果园环境

clear all;
clc;

figure('Name', '简化果园环境对比', 'Position', [50, 100, 1500, 400]);

%% 1. 极简版 (3棵树)
subplot(1, 3, 1);
env1 = simpleOrchard('minimal');
visualizeOrchardEnv(env1);
plot3(env1.source(1), env1.source(2), env1.source(3), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
plot3(env1.goal(1), env1.goal(2), env1.goal(3), 'r^', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
title(sprintf('极简版 (3棵树)\n起点→终点距离: %.0fmm', norm(env1.source - env1.goal)));
legend('树', '起点', '终点', 'Location', 'best');

%% 2. 标准版 (5棵树)
subplot(1, 3, 2);
env2 = simpleOrchard('standard');
visualizeOrchardEnv(env2);
plot3(env2.source(1), env2.source(2), env2.source(3), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
plot3(env2.goal(1), env2.goal(2), env2.goal(3), 'r^', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
plot3([env2.source(1), env2.goal(1)], [env2.source(2), env2.goal(2)], ...
      [env2.source(3), env2.goal(3)], 'g--', 'LineWidth', 1);
title(sprintf('标准版 (5棵树)\n起点→终点距离: %.0fmm', norm(env2.source - env2.goal)));
legend('树', '起点', '终点', '理想直线路径', 'Location', 'best');

%% 3. 稀疏版 (7棵树)
subplot(1, 3, 3);
env3 = simpleOrchard('sparse');
visualizeOrchardEnv(env3);
plot3(env3.source(1), env3.source(2), env3.source(3), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
plot3(env3.goal(1), env3.goal(2), env3.goal(3), 'r^', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
plot3([env3.source(1), env3.goal(1)], [env3.source(2), env3.goal(2)], ...
      [env3.source(3), env3.goal(3)], 'g--', 'LineWidth', 1);
% 标注两排树
for i = 1:3
    text(env3.obstacles(i,1), env3.obstacles(i,2), env3.obstacles(i,3)+100, '第1排', 'Color', 'w', 'FontSize', 8);
end
for i = 4:6
    text(env3.obstacles(i,1), env3.obstacles(i,2), env3.obstacles(i,3)+100, '第2排', 'Color', 'w', 'FontSize', 8);
end
title(sprintf('稀疏版 (7棵树-两排布局)\n起点→终点距离: %.0fmm', norm(env3.source - env3.goal)));
legend('树', '起点', '终点', '理想直线路径', 'Location', 'best');

%% 打印对比信息
fprintf('\n========== 简化果园环境对比 ==========\n\n');

variants = {'minimal', 'standard', 'sparse'};
titles = {'极简版', '标准版', '稀疏版'};

for i = 1:3
    env = simpleOrchard(variants{i});
    fprintf('【%s】 (%s)\n', titles{i}, variants{i});
    fprintf('  树木数量: %d\n', size(env.obstacles, 1));
    fprintf('  起点: [%.0f, %.0f, %.0f]\n', env.source);
    fprintf('  终点: [%.0f, %.0f, %.0f]\n', env.goal);
    fprintf('  工作空间: X[%.0f,%.0f] Y[%.0f,%.0f] Z[%.0f,%.0f]\n', ...
        env.bounds.xMin, env.bounds.xMax, env.bounds.yMin, env.bounds.yMax, ...
        env.bounds.zMin, env.bounds.zMax);
    fprintf('  安全余量: %dmm\n', env.safetyMargin);
    fprintf('\n');
end

fprintf('使用方式:\n');
fprintf('  env = simpleOrchard(''minimal'');   %% 快速测试\n');
fprintf('  env = simpleOrchard(''standard'');  %% 平衡 (推荐)\n');
fprintf('  env = simpleOrchard(''sparse'');    %% 复杂场景\n');
fprintf('\n然后运行: main_simple_orchard\n');
fprintf('========================================\n');

%% 可视化辅助函数
function visualizeOrchardEnv(env)
    cla;
    hold on;
    
    % 绘制树(简化椭球)
    [xs, ys, zs] = sphere(12);
    for i = 1:size(env.obstacles, 1)
        r = env.obsRadius(i);
        h = env.obsHeight(i);
        x = xs * r + env.obstacles(i, 1);
        y = ys * r + env.obstacles(i, 2);
        z = zs * (h * 0.5) + env.obstacles(i, 3) * 0.3;
        surf(x, y, z, 'FaceColor', env.treeColor, 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    end
    
    % 地面
    fill3([env.bounds.xMin, env.bounds.xMax, env.bounds.xMax, env.bounds.xMin], ...
          [env.bounds.yMin, env.bounds.yMin, env.bounds.yMax, env.bounds.yMax], ...
          [0, 0, 0, 0], env.groundColor, 'FaceAlpha', 0.4);
    
    % 安全区域(透明球)
    for i = 1:size(env.obstacles, 1)
        r = env.obsRadius(i) + env.safetyMargin;
        [xs, ys, zs] = sphere(8);
        x = xs * r + env.obstacles(i, 1);
        y = ys * r + env.obstacles(i, 2);
        z = zs * r + env.obstacles(i, 3);
        surf(x, y, z, 'FaceColor', 'r', 'FaceAlpha', 0.05, 'EdgeColor', 'none');
    end
    
    axis([env.bounds.xMin, env.bounds.xMax, ...
          env.bounds.yMin, env.bounds.yMax, ...
          env.bounds.zMin, env.bounds.zMax]);
    
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    axis equal; grid on;
    view(-35, 20);
    camlight; lighting gouraud;
end
