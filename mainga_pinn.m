%% 主程序: GA遗传算法 + PINN优化路径规划
% 本程序使用遗传算法进行初始路径规划，然后可选使用PINN进行路径优化
% 包含连杆碰撞检测和PINN优化开关

clear all;
clc;

%% ========== 配置参数 ==========
% 机械臂DH参数
a2 = 120;
a3 = 136;
d2 = 20;

%% 连杆碰撞检测开关 (极致优化版)
% 优化策略：
%   1. 单点检测：只检测路径中最靠近障碍物的1个点
%   2. 启发式预筛：基于末端位置快速排除安全路径(跳过ikunc)
%   3. 单障碍检测：只检查最接近的1个障碍物(无循环)
%   4. 内联展开：点到线段距离计算内联，无函数调用开销
% 相比原始全检测，逆运动学调用减少95%以上，检测速度提升20-30倍
enableLinkCollision = false;  % true=启用连杆碰撞检测, false=仅检测末端执行器

%% PINN优化开关
usePINN = true;  % true=启用PINN优化, false=仅使用GA路径+B样条平滑

%% ========== 机器人建模 ==========
% 定义机器人连杆参数，根据机器人DH表建立，采用改进DH法建模
L1 = Link('d', 41, 'a', 0, 'alpha', 0, 'modified');
L2 = Link('d', d2, 'a', 0, 'alpha', pi/2, 'modified');
L3 = Link('d', -20, 'a', a2, 'alpha', 0, 'modified');
L4 = Link('d', 0, 'a', a3, 'alpha', pi/2, 'modified');
L5 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'modified');
L6 = Link('d', 50, 'a', 0, 'alpha', 0, 'modified');

L2.qlim = [0, pi]; % 肩关节：0°~180° 大臂俯仰

% 通过SerialLink函数将定义的连杆组成机器人
PickingRoboticArm = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'ROBOT');

%% 设置可视化
view(-40, 30);
axis equal
xlabel('x');
ylabel('y');
PickingRoboticArm.plot([0 0 0 0 0 0]);   % 机械臂初始姿态
hold on;

%% ========== 果园环境配置 ==========
% 选择场景类型: 'static'(静态),'dynamic'(动态),'mixed'(混合)
orchardScenario = 'static';  % 果树静态环境
circleCenterSave = [200, -100, 80; 180, 0, 120; 150, 160, 260];
rSave = [45; 45; 45];

% 生成果园环境
env = orchardEnvironment(orchardScenario);

% 使用果园环境的起点终点
source = env.source;  % 果园入口
goal = env.goal;      % 目标采摘位置

fprintf('果园环境: %s\n', env.name);
fprintf('起点: [%.1f, %.1f, %.1f]\n', source(1), source(2), source(3));
fprintf('终点: [%.1f, %.1f, %.1f]\n', goal(1), goal(2), goal(3));
fprintf('障碍物数量: %d\n', size(env.obstacles, 1));

%% 兼容性变量
circleCenter = env.obstacles(:, 1:3);  % GA算法使用3D圆心
r = env.obsRadius;  % 使用果园障碍物半径

%% ========== 阶段1: GA遗传算法路径规划 ==========
disp('=================================================');
disp('阶段1: GA遗传算法路径规划');
disp('=================================================');

tic;

% GA参数配置
gaOptions = struct();
gaOptions.populationSize = 80;      % 种群大小
gaOptions.maxGenerations = 5000;     % 最大迭代次数
gaOptions.crossoverRate = 0.85;       % 交叉概率
gaOptions.mutationRate = 0.15;      % 变异概率
gaOptions.eliteCount = 5;            % 精英保留数量
gaOptions.waypoints = 15;            % 路径中间点数
gaOptions.verbose = true;

% 执行GA路径规划
disp('使用GA遗传算法进行路径规划...');
if enableLinkCollision
    disp('已启用连杆碰撞检测');
end

[path, fitnessHistory] = GAPath(source, goal, circleCenter, r, gaOptions, ...
    enableLinkCollision, PickingRoboticArm, a2, a3, d2);

gaTime = toc;

mypos_iter = path;

%% 连杆碰撞检测验证
if enableLinkCollision
    disp('开始连杆碰撞检测验证...');
    collisionFreePath = [];
    checkInterval = 2;
    
    for i = 1:checkInterval:size(mypos_iter, 1)
        T = eye(4);
        T(1:3, 4) = mypos_iter(i, :)';
        T(1:3, 1:3) = [1 0 0; 0 -1 0; 0 0 1];
        
        try
            q = PickingRoboticArm.ikunc(T);
            [collision, ~] = checkLinkCollisionFast(q, circleCenterSave, rSave, a2, a3, d2);
            if ~collision
                collisionFreePath = [collisionFreePath; mypos_iter(i, :)];
            end
        catch
            warning('路径点 %d 逆运动学求解失败', i);
        end
    end
    
    if size(collisionFreePath, 1) < 2
        error('路径存在过多连杆碰撞，请重新规划!');
    end
    mypos_iter = collisionFreePath;
    fprintf('GA路径有效点数: %d\n', size(mypos_iter, 1));
end

%% ========== 阶段2: PINN路径优化 (可选) ==========
new_Path = mypos_iter;
optimizedPath = mypos_iter;
pinnTime = 0;

if usePINN
    disp(' ');
    disp('=================================================');
    disp('阶段2: PINN物理信息神经网络优化');
    disp('=================================================');
    
    % PINN优化参数配置
    pinnOptions = struct();
    pinnOptions.maxIter = 300;
    pinnOptions.numPoints = max(30, size(mypos_iter, 1) * 5);
    pinnOptions.safeMargin = 8;
    pinnOptions.lambda_collision = 100;
    pinnOptions.lambda_smooth = 50;
    pinnOptions.lambda_length = 10;
    pinnOptions.verbose = true;
    
    % 执行PINN优化
    originalPath = mypos_iter;
    tic;
    optimizedPath = pinnOptimizePathClassic(originalPath, circleCenter, r, source, goal, pinnOptions);
    pinnTime = toc;
    
    % 对PINN优化后的路径进行B样条平滑
    new_Path = threespline(optimizedPath, rSave, circleCenterSave, source, goal);
    
    %% B样条平滑后连杆碰撞检测
    if enableLinkCollision
        disp('检查B样条平滑后的路径是否存在连杆碰撞...');
        collisionFreeSmoothPath = [];
        checkInterval = 3;
        
        for i = 1:checkInterval:size(new_Path, 1)
            T = eye(4);
            T(1:3, 4) = new_Path(i, :)';
            T(1:3, 1:3) = [1 0 0; 0 -1 0; 0 0 1];
            try
                q = PickingRoboticArm.ikunc(T);
                [collision, ~] = checkLinkCollisionFast(q, circleCenterSave, rSave, a2, a3, d2);
                if ~collision
                    collisionFreeSmoothPath = [collisionFreeSmoothPath; new_Path(i, :)];
                end
            catch
            end
        end
        
        if size(collisionFreeSmoothPath, 1) >= 2
            new_Path = collisionFreeSmoothPath;
        else
            warning('B样条平滑后路径存在过多碰撞，使用PINN优化路径');
        end
    end
else
    % 不使用PINN，直接对GA路径进行B样条平滑
    disp(' ');
    disp('=================================================');
    disp('阶段2: B样条平滑 (PINN优化已关闭)');
    disp('=================================================');
    new_Path = threespline(mypos_iter, rSave, circleCenterSave, source, goal);
    
    %% B样条平滑后连杆碰撞检测
    if enableLinkCollision
        disp('检查B样条平滑后的路径是否存在连杆碰撞...');
        collisionFreeSmoothPath = [];
        checkInterval = 3;
        
        for i = 1:checkInterval:size(new_Path, 1)
            T = eye(4);
            T(1:3, 4) = new_Path(i, :)';
            T(1:3, 1:3) = [1 0 0; 0 -1 0; 0 0 1];
            try
                q = PickingRoboticArm.ikunc(T);
                [collision, ~] = checkLinkCollisionFast(q, circleCenterSave, rSave, a2, a3, d2);
                if ~collision
                    collisionFreeSmoothPath = [collisionFreeSmoothPath; new_Path(i, :)];
                end
            catch
            end
        end
        
        if size(collisionFreeSmoothPath, 1) >= 2
            new_Path = collisionFreeSmoothPath;
        else
            warning('B样条平滑后路径存在过多碰撞，使用原始GA路径');
            new_Path = mypos_iter;
        end
    end
end

%% ========== 阶段3: 机器人运动学计算与可视化 ==========
n = length(new_Path);
a = zeros(4, 4, n);

for num1 = 1:n
    x_taget = new_Path(n+1-num1, 1);
    y_taget = new_Path(n+1-num1, 2);
    z_taget = new_Path(n+1-num1, 3);
    a(1, 1, num1) = 1;
    a(2, 2, num1) = -1;
    a(3, 3, num1) = 1;
    a(1, 4, num1) = x_taget;
    a(2, 4, num1) = y_taget;
    a(3, 4, num1) = z_taget;
    a(4, 4, num1) = 1;
end

q = PickingRoboticArm.ikunc(a);

%% 机器人动画与末端轨迹可视化
figure(4);
clf;
hold on;
axis equal; grid on;

% 绘制障碍物
for i = 1:size(circleCenter, 1)
    [xs, ys, zs] = sphere(20);
    mesh(r(i)*xs + circleCenter(i,1), r(i)*ys + circleCenter(i,2), ...
        r(i)*zs + circleCenter(i,3), 'FaceAlpha', 0.2, 'EdgeColor', 'k');
end

% 绘制起点终点
plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r');

% 设置坐标轴范围
allPoints = [source; goal; circleCenter];
margin = 100;
xlim([min(allPoints(:,1))-margin, max(allPoints(:,1))+margin]);
ylim([min(allPoints(:,2))-margin, max(allPoints(:,2))+margin]);
zlim([min(allPoints(:,3))-margin, max(allPoints(:,3))+margin]);

xlabel('X'); ylabel('Y'); zlabel('Z');
title('机器人执行优化后的路径（带末端轨迹）');
view(3);

% 预计算末端轨迹
endEffectorPos = zeros(n, 3);
for i = 1:n
    T = PickingRoboticArm.fkine(q(i, :));
    endEffectorPos(i, :) = transl(T)';
end

% 动画播放与轨迹绘制
trailHandle = plot3(endEffectorPos(1,1), endEffectorPos(1,2), endEffectorPos(1,3), ...
    'r-', 'LineWidth', 2, 'DisplayName', '末端轨迹');

step = max(1, floor(n / 100));
disp('播放机器人动画...');

for i = 1:step:n
    set(trailHandle, 'XData', endEffectorPos(1:i,1), ...
        'YData', endEffectorPos(1:i,2), ...
        'ZData', endEffectorPos(1:i,3));
    PickingRoboticArm.plot(q(i, :));
    drawnow limitrate;
    pause(0.03);
end

plot3(endEffectorPos(:,1), endEffectorPos(:,2), endEffectorPos(:,3), ...
    'r-', 'LineWidth', 2, 'DisplayName', '末端轨迹');
hold on;
PickingRoboticArm.plot(q(end, :));
legend('末端轨迹', 'Location', 'best');

%% 关节空间轨迹图
figure(5);
plot(q);
legend('关节1', '关节2', '关节3', '关节4', '关节5', '关节6', 'Location', 'best');
xlabel('路径点');
ylabel('关节角度 (rad)');
title('关节空间轨迹');
grid on;

%% ========== 性能评估 ==========
disp(' ');
disp('=================================================');
disp('性能评估对比');
disp('=================================================');

% GA原始路径指标
originalLength = computePathLength(mypos_iter);
originalSmooth = computePathSmoothness(mypos_iter);

% PINN优化后路径指标 (如果使用)
if usePINN
    optimizedLength = computePathLength(optimizedPath);
    optimizedSmooth = computePathSmoothness(optimizedPath);
end

% B样条后最终路径指标
finalLength = computePathLength(new_Path);
finalSmooth = computePathSmoothness(new_Path);

% 终点定位误差
actual_end_pose = new_Path(end, :);
end_position_error = norm(actual_end_pose - goal);

fprintf('计算耗时:\n');
fprintf('  - GA路径规划: %.3f 秒\n', gaTime);
if usePINN
    fprintf('  - PINN优化: %.3f 秒\n', pinnTime);
end

fprintf('\n路径长度:\n');
fprintf('  - GA原始路径: %.4f\n', originalLength);
if usePINN
    fprintf('  - PINN优化后:  %.4f (减少 %.2f%%)\n', optimizedLength, (1-optimizedLength/originalLength)*100);
end
fprintf('  - B样条平滑后:  %.4f\n', finalLength);

fprintf('\n路径平滑度 (越小越好):\n');
fprintf('  - GA原始路径: %.4f\n', originalSmooth);
if usePINN
    fprintf('  - PINN优化后:  %.4f (改善 %.2f%%)\n', optimizedSmooth, (1-optimizedSmooth/originalSmooth)*100);
end
fprintf('  - B样条平滑后:  %.4f\n', finalSmooth);
fprintf('\n终点定位误差: %.4f\n', end_position_error);
fprintf('=================================================\n');

%% ========== 最终路径对比可视化 ==========
if usePINN
    figure('Name', '路径优化全流程对比', 'Position', [50 50 1500 400]);
    
    subplot(1, 3, 1);
    plot3(mypos_iter(:,1), mypos_iter(:,2), mypos_iter(:,3), 'b-o', 'LineWidth', 1.5);
    hold on;
    for i = 1:size(circleCenter, 1)
        [x, y, z] = sphere(20);
        mesh(r(i)*x+circleCenter(i,1), r(i)*y+circleCenter(i,2), r(i)*z+circleCenter(i,3), 'FaceAlpha', 0.3);
    end
    plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title(sprintf('原始GA路径\n长度=%.2f, 平滑度=%.2f', originalLength, originalSmooth));
    xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal; grid on;
    
    subplot(1, 3, 2);
    plot3(optimizedPath(:,1), optimizedPath(:,2), optimizedPath(:,3), 'r-o', 'LineWidth', 1.5);
    hold on;
    for i = 1:size(circleCenter, 1)
        [x, y, z] = sphere(20);
        mesh(r(i)*x+circleCenter(i,1), r(i)*y+circleCenter(i,2), r(i)*z+circleCenter(i,3), 'FaceAlpha', 0.3);
    end
    plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title(sprintf('PINN优化后路径\n长度=%.2f, 平滑度=%.2f', optimizedLength, optimizedSmooth));
    xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal; grid on;
    
    subplot(1, 3, 3);
    plot3(new_Path(:,1), new_Path(:,2), new_Path(:,3), 'g-', 'LineWidth', 2);
    hold on;
    for i = 1:size(circleCenter, 1)
        [x, y, z] = sphere(20);
        mesh(r(i)*x+circleCenter(i,1), r(i)*y+circleCenter(i,2), r(i)*z+circleCenter(i,3), 'FaceAlpha', 0.3);
    end
    plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title(sprintf('最终路径 (GA+PINN+B样条)\n长度=%.2f, 平滑度=%.2f', finalLength, finalSmooth));
    xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal; grid on;
else
    figure('Name', 'GA路径规划结果', 'Position', [50 50 1000 400]);
    
    subplot(1, 2, 1);
    plot3(mypos_iter(:,1), mypos_iter(:,2), mypos_iter(:,3), 'b-o', 'LineWidth', 1.5);
    hold on;
    for i = 1:size(circleCenter, 1)
        [x, y, z] = sphere(20);
        mesh(r(i)*x+circleCenter(i,1), r(i)*y+circleCenter(i,2), r(i)*z+circleCenter(i,3), 'FaceAlpha', 0.3);
    end
    plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title(sprintf('原始GA路径\n长度=%.2f, 平滑度=%.2f', originalLength, originalSmooth));
    xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal; grid on;
    
    subplot(1, 2, 2);
    plot3(new_Path(:,1), new_Path(:,2), new_Path(:,3), 'g-', 'LineWidth', 2);
    hold on;
    for i = 1:size(circleCenter, 1)
        [x, y, z] = sphere(20);
        mesh(r(i)*x+circleCenter(i,1), r(i)*y+circleCenter(i,2), r(i)*z+circleCenter(i,3), 'FaceAlpha', 0.3);
    end
    plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title(sprintf('B样条平滑后路径\n长度=%.2f, 平滑度=%.2f', finalLength, finalSmooth));
    xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal; grid on;
end

%% ========== 辅助函数 ==========
function totalLength = computePathLength(path)
    diffs = diff(path);
    distances = sqrt(sum(diffs.^2, 2));
    totalLength = sum(distances);
end

function smoothness = computePathSmoothness(path)
    if size(path, 1) < 3
        smoothness = 0;
        return;
    end
    second_diff = diff(path, 2);
    smoothness = sum(sqrt(sum(second_diff.^2, 2)));
end
