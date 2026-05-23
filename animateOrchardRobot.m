function animateOrchardRobot(env, q, path, PickingRoboticArm, options)
%% 机械臂果园作业动画演示
% 输入:
%   env - 果园环境
%   q - 关节角度序列 [N×6]
%   path - 末端执行器路径 [N×3]
%   PickingRoboticArm - 机器人模型
%   options - 动画参数结构体 (可选)

if nargin < 5
    options = struct();
end

% 默认参数
if ~isfield(options, 'fps'), options.fps = 30; end
if ~isfield(options, 'loop'), options.loop = false; end
if ~isfield(options, 'trailLength'), options.trailLength = 50; end
if ~isfield(options, 'saveVideo'), options.saveVideo = false; end
if ~isfield(options, 'videoName'), options.videoName = 'orchard_robot_animation.mp4'; end
if ~isfield(options, 'showPath'), options.showPath = true; end
if ~isfield(options, 'showWorkspace'), options.showWorkspace = true; end
if ~isfield(options, 'camFollow'), options.camFollow = true; end

n = size(q, 1);
if n < 2
    warning('关节角度序列太短，无法动画');
    return;
end

%% 创建图形窗口
fig = figure('Name', '机械臂果园作业动画', 'Position', [100, 100, 1200, 800]);

% 设置光照
light('Position', [1 1 1], 'Style', 'infinite');
lighting gouraud;

%% 初始化绘图
clf;

% 1. 绘制果园环境 (静态)
subplot(2, 2, [1, 3]);
hold on;
plotOrchardEnvironment(env);

% 绘制完整路径轨迹
if options.showPath
    plot3(path(:,1), path(:,2), path(:,3), 'c--', 'LineWidth', 1, 'DisplayName', '规划路径');
end

% 初始化机械臂图形
PickingRoboticArm.plot(q(1, :), 'workspace', [env.bounds.xMin, env.bounds.xMax, ...
    env.bounds.yMin, env.bounds.yMax, env.bounds.zMin, env.bounds.zMax]);

% 末端轨迹线
if options.trailLength > 0
    trailHandle = plot3(path(1,1), path(1,2), path(1,3), 'r-', 'LineWidth', 2);
end

% 当前位置标记
currentPosHandle = plot3(path(1,1), path(1,2), path(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% 标注信息
titleHandle = title(sprintf('果园作业仿真 - 帧 %d/%d', 1, n));
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
axis equal; grid on;
view(-40, 25);

% 2. 关节角度实时显示
subplot(2, 2, 2);
jointPlotHandles = zeros(6, 1);
colors = lines(6);
legendNames = {'关节1', '关节2', '关节3', '关节4', '关节5', '关节6'};
for j = 1:6
    jointPlotHandles(j) = plot(1:n, q(:,j), 'Color', colors(j,:), 'LineWidth', 1.5);
    hold on;
end
currentJointLine = xline(1, 'r--', 'LineWidth', 2);
xlim([1, n]);
xlabel('帧');
ylabel('关节角度 (rad)');
title('关节角度变化');
legend(legendNames, 'Location', 'best');
grid on;

% 3. 末端位置坐标显示
subplot(2, 2, 4);
positionData = [path, sqrt(sum(path.^2, 2))]; % [x, y, z, dist]
posPlotHandles = zeros(4, 1);
posLabels = {'X', 'Y', 'Z', '距离'};
posColors = ['r', 'g', 'b', 'm'];
for j = 1:4
    posPlotHandles(j) = plot(1:n, positionData(:,j), 'Color', posColors(j), 'LineWidth', 1.5);
    hold on;
end
currentPosLine = xline(1, 'r--', 'LineWidth', 2);
xlim([1, n]);
xlabel('帧');
ylabel('位置 (mm)');
title('末端执行器位置');
legend(posLabels, 'Location', 'best');
grid on;

%% 视频录制准备
if options.saveVideo
    try
        v = VideoWriter(options.videoName, 'MPEG-4');
        v.FrameRate = options.fps;
        v.Quality = 95;
        open(v);
        recording = true;
    catch
        warning('视频录制初始化失败，仅播放动画');
        recording = false;
    end
else
    recording = false;
end

%% 动画循环
dt = 1 / options.fps;
frameCount = 0;

fprintf('\n>>> 动画播放中... (按Ctrl+C中断)\n');

try
    while true
        for i = 1:n
            tic;
            
            % 更新机械臂姿态
            PickingRoboticArm.animate(q(i, :));
            
            % 更新末端轨迹
            if options.trailLength > 0
                startIdx = max(1, i - options.trailLength);
                set(trailHandle, 'XData', path(startIdx:i, 1), ...
                    'YData', path(startIdx:i, 2), ...
                    'ZData', path(startIdx:i, 3));
            end
            
            % 更新当前位置标记
            set(currentPosHandle, 'XData', path(i, 1), ...
                'YData', path(i, 2), ...
                'ZData', path(i, 3));
            
            % 更新标题
            progress = (i / n) * 100;
            set(titleHandle, 'String', ...
                sprintf('果园作业仿真 - 帧 %d/%d (%.1f%%)', i, n, progress));
            
            % 更新关节角度指示线
            set(currentJointLine, 'Value', i);
            set(currentPosLine, 'Value', i);
            
            % 相机跟随
            if options.camFollow && i > 1
                currentPos = path(i, :);
                campos([currentPos(1) - 300, currentPos(2) - 300, currentPos(3) + 200]);
                camtarget(currentPos);
            end
            
            % 录制帧
            if recording
                frame = getframe(fig);
                writeVideo(v, frame);
            end
            
            drawnow;
            
            % 控制帧率
            elapsed = toc;
            if elapsed < dt
                pause(dt - elapsed);
            end
            
            frameCount = frameCount + 1;
        end
        
        if ~options.loop
            break;
        end
        
        fprintf('>>> 循环播放 (按Ctrl+C停止)\n');
    end
    
catch ME
    if strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
        fprintf('>>> 图形窗口已关闭\n');
    else
        fprintf('>>> 动画中断: %s\n', ME.message);
    end
end

%% 结束录制
if recording
    close(v);
    fprintf('>>> 视频已保存: %s\n', options.videoName);
end

fprintf('>>> 动画结束，共播放 %d 帧\n', frameCount);

%% 最终静态展示
figure('Name', '运动轨迹总结', 'Position', [150, 150, 1000, 400]);

% 轨迹动画回放静态图
subplot(1, 2, 1);
plotOrchardEnvironment(env);
hold on;
plot3(path(:,1), path(:,2), path(:,3), 'c--', 'LineWidth', 1);
scatter3(path(:,1), path(:,2), path(:,3), 10, 1:n, 'filled');
colormap(jet);
cb = colorbar;
ylabel(cb, '帧序号');
plot3(env.source(1), env.source(2), env.source(3), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot3(env.goal(1), env.goal(2), env.goal(3), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
title('末端执行器轨迹 (颜色=时间)');
axis equal; view(-40, 25);

% 关节速度
subplot(1, 2, 2);
jointVel = diff(q) * options.fps; % rad/s
for j = 1:6
    plot(1:(n-1), jointVel(:,j), 'Color', colors(j,:), 'LineWidth', 1.5);
    hold on;
end
xlabel('帧');
ylabel('关节速度 (rad/s)');
title('关节速度曲线');
legend(legendNames, 'Location', 'best');
grid on;

end

%% 辅助函数：绘制果园环境
function plotOrchardEnvironment(env)
    % 绘制树
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
    
    % 起点终点
    plot3(env.source(1), env.source(2), env.source(3), 'bs', ...
        'MarkerSize', 15, 'MarkerFaceColor', 'b', 'DisplayName', '起点');
    plot3(env.goal(1), env.goal(2), env.goal(3), 'r^', ...
        'MarkerSize', 15, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
    
    % 安全区域
    for i = 1:size(env.obstacles, 1)
        r = env.obsRadius(i) + env.safetyMargin;
        [xs, ys, zs] = sphere(6);
        x = xs * r + env.obstacles(i, 1);
        y = ys * r + env.obstacles(i, 2);
        z = zs * r + env.obstacles(i, 3);
        surf(x, y, z, 'FaceColor', 'r', 'FaceAlpha', 0.05, 'EdgeColor', 'none');
    end
    
    axis([env.bounds.xMin, env.bounds.xMax, ...
          env.bounds.yMin, env.bounds.yMax, ...
          env.bounds.zMin, env.bounds.zMax]);
    
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    grid on;
    camlight; lighting gouraud;
end
