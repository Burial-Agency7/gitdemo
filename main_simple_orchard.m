%% 简化果园环境路径规划演示
% 机械臂在简化果园中避障路径规划

clear all;
clc;

%% ========== 1. 机器人建模 ==========
% 果园采摘机械臂 - 改进DH参数定义
% 连杆长度参数 (单位: mm)
a2 = 120;   % 大臂长度 (Link 2-3)
a3 = 136;   % 小臂长度 (Link 3-4)
d2 = 20;    % 关节2偏置
d1 = 41;    % 基座高度 (标准高度)

% 定义6个连杆 (改进DH法)
% 格式: Link('d', d_i, 'a', a_{i-1}, 'alpha', alpha_{i-1}, 'modified')

% Link 1: 基座旋转关节 (腰关节)
L1 = Link('d', d1, 'a', 0, 'alpha', 0, 'modified');
L1.qlim = deg2rad([-160, 160]);  % 腰关节限位: ±160°

% Link 2: 肩关节 (大臂俯仰)
L2 = Link('d', d2, 'a', 0, 'alpha', pi/2, 'modified');
L2.qlim = deg2rad([-90, 90]);   % 肩关节限位: ±90°

% Link 3: 肘关节 (小臂俯仰)
L3 = Link('d', -20, 'a', a2, 'alpha', 0, 'modified');
L3.qlim = deg2rad([-160, 160]); % 肘关节限位: ±160°

% Link 4: 腕关节偏航
L4 = Link('d', 0, 'a', a3, 'alpha', pi/2, 'modified');
L4.qlim = deg2rad([-170, 170]); % 腕偏航限位: ±170°

% Link 5: 腕关节俯仰
L5 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'modified');
L5.qlim = deg2rad([-85, 85]);   % 腕俯仰限位: ±85°

% Link 6: 末端执行器旋转
L6 = Link('d', 50, 'a', 0, 'alpha', 0, 'modified');
L6.qlim = deg2rad([-170, 170]); % 末端旋转限位: ±170°

% 组装机械臂模型
PickingRoboticArm = SerialLink([L1 L2 L3 L4 L5 L6], ...
    'name', 'PickingRoboticArm', ...
    'comment', 'Orchard Picking Robot Arm v1.0');

% 显示机械臂基本信息
fprintf('\n========== 机械臂模型 ==========\n');
fprintf('名称: %s\n', PickingRoboticArm.name);
fprintf('自由度: %d\n', PickingRoboticArm.n);
fprintf('基座高度: %.1f mm\n', d1);
fprintf('大臂长度: %.1f mm\n', a2);
fprintf('小臂长度: %.1f mm\n', a3);

%% ========== 2. 简化果园环境 ==========
orchardVariant = 'sparse';  % 'minimal', 'standard', 'sparse'
env = simpleOrchard(orchardVariant);

source = env.source;
goal = env.goal;

fprintf('\n========== 简化果园路径规划 ==========\n');
fprintf('环境版本: %s\n', orchardVariant);
fprintf('树木数量: %d\n', size(env.obstacles, 1));
fprintf('起点: [%.1f, %.1f, %.1f]\n', source);
fprintf('终点: [%.1f, %.1f, %.1f]\n', goal);

%% ========== 3. GA路径规划 ==========
disp(' ');
disp('>>> GA遗传算法路径规划...');

circleCenter = env.obstacles;
r = env.obsRadius;

% GA参数设置
gaOptions = struct();
gaOptions.populationSize = 80;
gaOptions.maxGenerations = 3000;
gaOptions.crossoverRate = 0.85;
gaOptions.mutationRate = 0.15;
gaOptions.waypoints = 15;  % 使用'waypoints'而非'numWaypoints'
gaOptions.prunePath = false;  % 禁用路径简化，保留更多点
gaOptions.verbose = true;

figure(1); clf;
[path, fitnessHistory] = GAPath(source, goal, circleCenter, r, gaOptions, false, PickingRoboticArm, a2, a3, d2);

mypos_iter = path;
fprintf('GA路径点数: %d\n', size(mypos_iter, 1));
fprintf('GA最优适应度: %.4f\n', fitnessHistory(end));

%% ========== 4. PINN优化 ==========
disp(' ');
disp('>>> PINN路径优化...');

pinnOptions = struct();
pinnOptions.maxIter = 800;
pinnOptions.numPoints = max(60, size(mypos_iter, 1) * 8);
pinnOptions.safeMargin = env.safetyMargin + 5;  % 适应缩小后的环境
pinnOptions.lambda_collision = 150;  % 降低碰撞权重
pinnOptions.lambda_smooth = 50;
pinnOptions.lambda_length = 5;
pinnOptions.verbose = false;

optimizedPath = pinnOptimizePathClassic(mypos_iter, circleCenter, r, source, goal, pinnOptions);
fprintf('PINN优化后路径点数: %d\n', size(optimizedPath, 1));

%% ========== 5. 连杆碰撞检测 (简化版) ==========
disp(' ');
disp('>>> 连杆碰撞检测...');

new_Path = optimizedPath;
collisionCount = 0;

for i = 1:size(new_Path, 1)
    T = eye(4);
    T(1:3, 4) = new_Path(i, :)';
    T(1:3, 1:3) = [1 0 0; 0 -1 0; 0 0 1];
    
    try
        q = PickingRoboticArm.ikunc(T);
        [collision, ~] = checkLinkCollisionFast(q, env.obstacles, env.obsRadius, a2, a3, d2);
        if collision
            collisionCount = collisionCount + 1;
        end
    catch
        collisionCount = collisionCount + 1;
    end
end

fprintf('检测到碰撞点: %d/%d\n', collisionCount, size(new_Path, 1));

%% ========== 6. 逆运动学与可视化 ==========
disp(' ');
disp('>>> 计算关节轨迹...');

n = size(new_Path, 1);
q = zeros(n, 6);
q_prev = [0, -pi/4, pi/3, 0, pi/4, 0];  % 更好的初始猜测

ikineSuccess = 0;
ikineFail = 0;
joint3BelowGround = 0;

for i = 1:n
    idx = n + 1 - i;  % 从终点倒序
    T = eye(4);
    T(1:3, 4) = new_Path(idx, :)';
    T(1:3, 1:3) = [1 0 0; 0 -1 0; 0 0 1];
    
    q_curr = [];
    
    % 方法1: ikine with current guess
    try
        q_test = PickingRoboticArm.ikine(T, 'q0', q_prev, 'tol', 1e-3, 'ilimit', 200);
        if ~isempty(q_test) && ~any(isnan(q_test))
            % 检查关节3位置
            if checkJoint3AboveGround(q_test, PickingRoboticArm, a2, a3, d2)
                q_curr = q_test;
                ikineSuccess = ikineSuccess + 1;
            end
        end
    catch
    end
    
    % 方法2: 如果失败，尝试ikunc
    if isempty(q_curr)
        try
            q_test = PickingRoboticArm.ikunc(T);
            if ~isempty(q_test) && ~any(isnan(q_test))
                if checkJoint3AboveGround(q_test, PickingRoboticArm, a2, a3, d2)
                    q_curr = q_test;
                    ikineSuccess = ikineSuccess + 1;
                end
            end
        catch
        end
    end
    
    % 方法3: 尝试不同初始猜测
    if isempty(q_curr)
        guessList = [0, 0, 0, 0, 0, 0; 
                     0, -pi/2, pi/2, 0, 0, 0;
                     0, pi/4, -pi/4, 0, pi/2, 0;
                     0, -0.5, 1.0, 0, 0, 0;   % 高位姿猜测
                     0, -0.3, 0.8, 0, 0, 0];   % 中高位姿猜测
        for g = 1:size(guessList, 1)
            try
                q_test = PickingRoboticArm.ikine(T, 'q0', guessList(g,:), 'tol', 1e-3, 'ilimit', 200);
                if ~isempty(q_test) && ~any(isnan(q_test))
                    if checkJoint3AboveGround(q_test, PickingRoboticArm, a2, a3, d2)
                        q_curr = q_test;
                        ikineSuccess = ikineSuccess + 1;
                        break;
                    end
                end
            catch
                continue;
            end
        end
    end
    
    % 更新结果
    if ~isempty(q_curr)
        q(i, :) = q_curr;
        q_prev = q_curr;
    else
        q(i, :) = q_prev;  % 保持前一姿态
        ikineFail = ikineFail + 1;
        if ~isempty(q_test) && ~any(isnan(q_test))
            if ~checkJoint3AboveGround(q_test, PickingRoboticArm, a2, a3, d2)
                joint3BelowGround = joint3BelowGround + 1;
            end
        end
    end
end

fprintf('逆运动学: %d成功, %d失败 (使用前一姿态)\n', ikineSuccess, ikineFail);
if joint3BelowGround > 0
    fprintf('注意: %d个姿态因关节3低于地面被拒绝\n', joint3BelowGround);
end

%% ========== 6.5 关节角度平滑处理 ==========
disp(' ');
disp('>>> 平滑关节角度轨迹...');

n_points = size(q, 1);

% 检查点数是否足够
if n_points < 4
    fprintf('轨迹点数(%d)过少，跳过样条平滑\n', n_points);
else
    % 方法1: 移动平均平滑
    windowSize = min(5, n_points);  % 窗口大小不超过点数
    q_smoothed = q;
    for j = 1:6
        for i = windowSize:n_points
            q_smoothed(i, j) = mean(q(i-windowSize+1:i, j));
        end
    end
    
    % 方法2: 使用样条插值重新采样平滑
    t_original = linspace(0, 1, n_points);
    n_dense = max(n_points * 2, 10);  % 至少10个点
    t_dense = linspace(0, 1, n_dense);
    
    q_spline = zeros(length(t_dense), 6);
    for j = 1:6
        try
            q_spline(:, j) = spline(t_original, q_smoothed(:, j), t_dense);
        catch
            % 如果样条失败，使用线性插值
            q_spline(:, j) = interp1(t_original, q_smoothed(:, j), t_dense, 'linear');
        end
    end
    
    % 选择平滑后的轨迹
    q = q_spline;
    new_Path = interp1(linspace(0, 1, size(new_Path, 1)), new_Path, t_dense, 'linear');
    
    fprintf('关节角度已平滑处理，轨迹点数: %d -> %d\n', n_points, length(t_dense));
end

%% ========== 7. 综合可视化 ==========
figure(2); clf;
set(gcf, 'Position', [100, 100, 1400, 400]);

% 子图1: 原始RRT路径
subplot(1, 3, 1);
hold on;
axis([env.bounds.xMin, env.bounds.xMax, env.bounds.yMin, env.bounds.yMax, env.bounds.zMin, env.bounds.zMax]);
% 绘制障碍物
drawObstacles(env);
% 绘制路径（检查是否为空）
if size(mypos_iter, 1) >= 2
    plot3(mypos_iter(:,1), mypos_iter(:,2), mypos_iter(:,3), 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'RRT路径');
else
    warning('RRT路径数据不足，无法绘制');
end
plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
if size(mypos_iter, 1) >= 2
    title(sprintf('原始RRT路径\n长度=%.1f', computePathLength(mypos_iter)));
else
    title('原始RRT路径\n(数据不足)');
end
legend('Location', 'best');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
axis equal; grid on; view(-40, 25);

% 子图2: PINN优化后路径
subplot(1, 3, 2);
hold on;
axis([env.bounds.xMin, env.bounds.xMax, env.bounds.yMin, env.bounds.yMax, env.bounds.zMin, env.bounds.zMax]);
% 绘制障碍物
drawObstacles(env);
% 绘制路径（带空检查）
if size(optimizedPath, 1) >= 2
    plot3(optimizedPath(:,1), optimizedPath(:,2), optimizedPath(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'PINN优化路径');
end
if size(mypos_iter, 1) >= 2
    plot3(mypos_iter(:,1), mypos_iter(:,2), mypos_iter(:,3), 'b--', 'LineWidth', 1, 'DisplayName', '原始RRT路径');
end
plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
legend('障碍物(树)', 'PINN优化路径', '原始RRT路径', '起点', '终点', 'Location', 'best');
if size(optimizedPath, 1) >= 2
    title(sprintf('PINN优化后路径\n长度=%.1f, 平滑度=%.2f', ...
        computePathLength(optimizedPath), computePathSmoothness(optimizedPath)));
else
    title('PINN优化后路径\n(数据不足)');
end
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
axis equal; grid on; view(-40, 25);

% 子图3: 关节空间轨迹
subplot(1, 3, 3);
if size(q, 1) >= 2
    plot(q, 'LineWidth', 1.5);
    legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'Location', 'best');
    xlabel('路径点');
    ylabel('关节角度 (rad)');
    title(sprintf('关节空间轨迹\n平滑度=%.2f', computePathSmoothness(q)));
else
    text(0.5, 0.5, '关节轨迹数据不足', 'HorizontalAlignment', 'center');
    title('关节空间轨迹\n(数据不足)');
end
grid on;

%% ========== 8. 3D机器人仿真 ==========
figure(3); clf;
hold on;
axis([env.bounds.xMin, env.bounds.xMax, env.bounds.yMin, env.bounds.yMax, env.bounds.zMin, env.bounds.zMax]);
% 绘制障碍物
drawObstacles(env);
% 绘制路径和起终点（带空检查）
if size(new_Path, 1) >= 2
    plot3(new_Path(:,1), new_Path(:,2), new_Path(:,3), 'm-', 'LineWidth', 2, 'DisplayName', '执行路径');
end
plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
title(sprintf('机械臂果园作业仿真\n障碍物数量: %d', size(env.obstacles, 1)));
legend('障碍物(树)', '执行路径', '起点', '终点', 'Location', 'best');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
axis equal; grid on; view(-40, 25);

% 显示多个机械臂状态：开始、过程、终点（分开展示）
if size(q, 1) >= 1
    % 注意：逆运动学计算时q是倒序存储的！
    % q(1,:)对应new_Path终点，q(end,:)对应new_Path起点
    nq = size(q, 1);
    midIdx = round(nq / 2);  % 中间位置
    
    % 创建三个独立的figure窗口，确保每个都能完整显示
    
    % Figure 4: 起点状态
    figure(4); clf;
    set(gcf, 'Position', [100, 550, 550, 450], 'Name', '起点姿态');
    hold on;
    axis([env.bounds.xMin, env.bounds.xMax, env.bounds.yMin, env.bounds.yMax, env.bounds.zMin, env.bounds.zMax]);
    drawObstacles(env);
    if size(new_Path, 1) >= 2
        plot3(new_Path(:,1), new_Path(:,2), new_Path(:,3), 'm--', 'LineWidth', 1);
    end
    plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
    PickingRoboticArm.plot(q(end, :), 'linkcolor', [0.3, 0.5, 0.9], 'jointcolor', [0.3, 0.5, 0.9]);
    title(sprintf('起点姿态 (路径起点)'), 'Color', [0.3, 0.5, 0.9], 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    axis equal; grid on; view(-40, 25);
    legend('障碍物(树)', '执行路径', '起点', '终点', 'Location', 'best');
    
    % Figure 5: 过程状态
    if nq >= 3
        figure(5); clf;
        set(gcf, 'Position', [680, 550, 550, 450], 'Name', '过程姿态');
        hold on;
        axis([env.bounds.xMin, env.bounds.xMax, env.bounds.yMin, env.bounds.yMax, env.bounds.zMin, env.bounds.zMax]);
        drawObstacles(env);
        if size(new_Path, 1) >= 2
            plot3(new_Path(:,1), new_Path(:,2), new_Path(:,3), 'm--', 'LineWidth', 1);
        end
        plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
        plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
        PickingRoboticArm.plot(q(midIdx, :), 'linkcolor', [0.2, 0.6, 0.3], 'jointcolor', [0.2, 0.6, 0.3]);
        title(sprintf('过程姿态 (路径中点)'), 'Color', [0.2, 0.6, 0.3], 'FontSize', 12, 'FontWeight', 'bold');
        xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
        axis equal; grid on; view(-40, 25);
        legend('障碍物(树)', '执行路径', '起点', '终点', 'Location', 'best');
    end
    
    % Figure 6: 终点状态
    if nq >= 2
        figure(6); clf;
        set(gcf, 'Position', [100, 50, 550, 450], 'Name', '终点姿态');
        hold on;
        axis([env.bounds.xMin, env.bounds.xMax, env.bounds.yMin, env.bounds.yMax, env.bounds.zMin, env.bounds.zMax]);
        drawObstacles(env);
        if size(new_Path, 1) >= 2
            plot3(new_Path(:,1), new_Path(:,2), new_Path(:,3), 'm--', 'LineWidth', 1);
        end
        plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
        plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
        PickingRoboticArm.plot(q(1, :), 'linkcolor', [0.8, 0.2, 0.2], 'jointcolor', [0.8, 0.2, 0.2]);
        title(sprintf('终点姿态 (路径终点)'), 'Color', [0.8, 0.2, 0.2], 'FontSize', 12, 'FontWeight', 'bold');
        xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
        axis equal; grid on; view(-40, 25);
        legend('障碍物(树)', '执行路径', '起点', '终点', 'Location', 'best');
    end
    
    fprintf('已生成三个关键姿态独立窗口 (Figure 4, 5, 6)\n');
else
    warning('无关节数据，无法显示机械臂');
end

fprintf('\n========== 规划完成 ==========\n');
fprintf('最终路径长度: %.2f mm\n', computePathLength(new_Path));
fprintf('关节轨迹平滑度: %.4f\n', computePathSmoothness(q));
fprintf('碰撞点数: %d\n', collisionCount);

%% ========== 8. 机械臂运动动画 ==========
disp(' ');
disp('>>> 启动运动动画演示...');

% 动画选项
animOptions = struct();
animOptions.fps = 30;              % 帧率
animOptions.loop = false;        % 不循环播放
animOptions.trailLength = 30;      % 轨迹拖尾长度
animOptions.saveVideo = false;     % 不保存视频(设为true可保存MP4)
animOptions.videoName = sprintf('orchard_%s_animation.mp4', orchardVariant);
animOptions.showPath = true;       % 显示规划路径
animOptions.camFollow = true;      % 相机跟随机械臂

% 播放动画
animateOrchardRobot(env, q, new_Path, PickingRoboticArm, animOptions);

%% ========== 辅助函数 ==========
function L = computePathLength(path)
    L = 0;
    for i = 2:size(path, 1)
        L = L + norm(path(i,:) - path(i-1,:));
    end
end

function S = computePathSmoothness(path)
    if size(path, 1) < 3
        S = 0;
        return;
    end
    second_diff = diff(path, 2);
    S = sum(sqrt(sum(second_diff.^2, 2)));
end

function drawObstacles(env)
    %% 绘制所有障碍物(树)
    % 绘制树(简化椭球形)
    [xs, ys, zs] = sphere(12);
    for i = 1:size(env.obstacles, 1)
        r = env.obsRadius(i);
        h = env.obsHeight(i);
        x = xs * r + env.obstacles(i, 1);
        y = ys * r + env.obstacles(i, 2);
        z = zs * (h * 0.5) + env.obstacles(i, 3) * 0.3;  % 压扁模拟树冠
        % 只在第一个障碍物设置图例标签，避免重复
        if i == 1
            surf(x, y, z, 'FaceColor', env.treeColor, 'FaceAlpha', 0.6, 'EdgeColor', 'none', 'DisplayName', '障碍物(树)');
        else
            surf(x, y, z, 'FaceColor', env.treeColor, 'FaceAlpha', 0.6, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
    end
    
    % 地面
    fill3([env.bounds.xMin, env.bounds.xMax, env.bounds.xMax, env.bounds.xMin], ...
          [env.bounds.yMin, env.bounds.yMin, env.bounds.yMax, env.bounds.yMax], ...
          [0, 0, 0, 0], env.groundColor, 'FaceAlpha', 0.4, 'EdgeColor', 'none');
    
    % 安全区域轮廓(透明)
    for i = 1:size(env.obstacles, 1)
        r = env.obsRadius(i) + env.safetyMargin;
        [xs, ys, zs] = sphere(6);
        x = xs * r + env.obstacles(i, 1);
        y = ys * r + env.obstacles(i, 2);
        z = zs * r + env.obstacles(i, 3);
        surf(x, y, z, 'FaceColor', 'r', 'FaceAlpha', 0.03, 'EdgeColor', 'none');
    end
    
    camlight; lighting gouraud;
end

function visualizeSimpleOrchard(env)
    %% 兼容旧代码：完整果园可视化
    axis([env.bounds.xMin, env.bounds.xMax, ...
          env.bounds.yMin, env.bounds.yMax, ...
          env.bounds.zMin, env.bounds.zMax]);
    hold on;
    
    % 绘制障碍物
    drawObstacles(env);
    
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    axis equal; grid on;
    view(-40, 25);
end

function isAbove = checkJoint3AboveGround(q, PickingRoboticArm, a2, a3, d2)
    %% 检查关节3是否高于地面(z>0)
    % 关节3是连杆3的起点(小臂根部)
    % 使用正运动学计算关节3位置
    % 注意：基座高度已改为71mm (原41+30)
    
    q1 = q(1); q2 = q(2); q3 = q(3);
    d1 = 71;  % 新的基座高度
    
    % 计算连杆变换
    % 关节1 (基座，d=71)
    T1 = [cos(q1), -sin(q1), 0, 0;
          sin(q1), cos(q1), 0, 0;
          0, 0, 1, d1;
          0, 0, 0, 1];
    
    % 关节2
    T2 = [cos(q2), -sin(q2), 0, 0;
          0, 0, -1, -d2;
          sin(q2), cos(q2), 0, 0;
          0, 0, 0, 1];
    
    % 关节3 (到关节3原点的变换)
    T3 = [cos(q3), -sin(q3), 0, a2;
          sin(q3), cos(q3), 0, 0;
          0, 0, 1, -20;
          0, 0, 0, 1];
    
    % 累积变换到关节3
    T_joint3 = T1 * T2 * T3;
    joint3_pos = T_joint3(1:3, 4);
    
    % 检查Z坐标 (基座已加高，关节3更容易高于地面)
    minZ = 10;  % 安全余量，要求高于10mm
    isAbove = joint3_pos(3) > minZ;
end
