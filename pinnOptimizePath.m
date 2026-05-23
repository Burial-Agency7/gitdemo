function [optimizedPath, varargout] = pinnOptimizePath(originalPath, circleCenter, r, source, goal, options)
%PINNOPTIMIZEPATH 使用物理信息神经网络(PINN)优化RRT+APF生成的路径
%
% 输入:
%   originalPath - 原始RRT路径点 [N x 3]
%   circleCenter - 障碍物中心坐标 [M x 3]
%   r            - 障碍物半径 [M x 1]
%   source       - 起点 [1 x 3]
%   goal         - 终点 [1 x 3]
%   options      - 优化参数结构体 (可选)
%
% 输出:
%   optimizedPath - PINN优化后的路径 [N x 3]
%
% PINN核心思想: 将物理约束(碰撞避免、路径平滑性、运动学约束)融入神经网络损失函数

%% 默认参数设置
if nargin < 6
    options = struct();
end
if ~isfield(options, 'hiddenLayers'), options.hiddenLayers = [128, 128, 64, 32]; end
if ~isfield(options, 'epochs'), options.epochs = 3000; end
if ~isfield(options, 'learningRate'), options.learningRate = 0.001; end
if ~isfield(options, 'lambda_collision'), options.lambda_collision = 10; end
if ~isfield(options, 'lambda_smooth'), options.lambda_smooth = 5; end
if ~isfield(options, 'lambda_length'), options.lambda_length = 2; end
if ~isfield(options, 'lambda_boundary'), options.lambda_boundary = 100; end
if ~isfield(options, 'numPoints'), options.numPoints = 100; end
if ~isfield(options, 'safeMargin'), options.safeMargin = 5; end
if ~isfield(options, 'verbose'), options.verbose = true; end

%% 数据预处理
N = size(originalPath, 1);
if N < 3
    error('路径点数量不足，至少需要3个点');
end

% 参数化路径 (使用累积弧长参数t ∈ [0,1])
t_original = computePathParameter(originalPath);

% 插值到固定数量的点用于训练
numTrainPoints = min(options.numPoints, N);
t_train = linspace(0, 1, numTrainPoints)';

% 三次样条插值获取训练数据
path_interp = interp1(t_original, originalPath, t_train, 'spline');

% 添加噪声增强鲁棒性
noise_level = 0.5;
path_noisy = path_interp + noise_level * randn(size(path_interp));

%% 构建PINN神经网络
% 网络架构: 输入(1D参数t) -> 隐藏层 -> 输出(3D坐标x,y,z)
numHidden = length(options.hiddenLayers);

layers = [
    featureInputLayer(1, 'Name', 'input', 'Normalization', 'none')
];

% 添加隐藏层
for i = 1:numHidden
    layers = [
        layers
        fullyConnectedLayer(options.hiddenLayers(i), 'Name', ['fc_' num2str(i)])
        reluLayer('Name', ['relu_' num2str(i)])
    ];
    % 添加Dropout防止过拟合
    if i < numHidden && mod(i, 2) == 0
        layers = [layers; dropoutLayer(0.1, 'Name', ['drop_' num2str(i)])];
    end
end

% 输出层 (3D坐标) - 注意：dlnetwork不需要regressionLayer
layers = [
    layers
    fullyConnectedLayer(3, 'Name', 'fc_output')
];

% 创建网络 - 使用dlnetwork直接构建
net = dlnetwork(layers);

%% 自定义PINN训练循环
% 使用ADAM优化器，结合物理约束

% 转换为dlarray (使用 'CB' 格式: Channel x Batch)
t_dl = dlarray(t_train', 'CB');  % [1 x numPoints]
path_target = dlarray(path_noisy', 'CB');  % [3 x numPoints]

% 初始化网络 (重要：需要先forward一次来初始化)
net = initialize(net, t_dl);

% 优化器参数
learnRate = options.learningRate;
gradDecay = 0.9;
sqGradDecay = 0.999;
avgGrad = [];
avgSqGrad = [];
iter = 0;

% 记录损失
lossHistory = struct('total', [], 'data', [], 'collision', [], 'smooth', [], 'length', [], 'boundary', []);

%% 训练循环
if options.verbose
    disp('开始PINN路径优化...');
    disp('物理约束: 碰撞避免 | 路径平滑 | 长度最小化 | 边界固定');
end

for epoch = 1:options.epochs
    iter = iter + 1;
    
    % 前向传播和损失计算
    [loss, gradients, lossComponents] = dlfeval(@modelLoss, net, t_dl, path_target, ...
        circleCenter, r, source, goal, options);
    
    % 更新网络参数
    [net, avgGrad, avgSqGrad] = adamupdate(net, gradients, avgGrad, avgSqGrad, iter, ...
        learnRate, gradDecay, sqGradDecay);
    
    % 记录损失
    lossHistory.total(epoch) = double(loss);
    lossHistory.data(epoch) = lossComponents.data;
    lossHistory.collision(epoch) = lossComponents.collision;
    lossHistory.smooth(epoch) = lossComponents.smooth;
    lossHistory.length(epoch) = lossComponents.length;
    lossHistory.boundary(epoch) = lossComponents.boundary;
    
    % 学习率衰减
    if mod(epoch, 500) == 0
        learnRate = learnRate * 0.9;
    end
    
    % 显示进度
    if options.verbose && mod(epoch, 500) == 0
        fprintf('Epoch %d/%d | Loss: %.4f | Data: %.4f | Collision: %.4f | Smooth: %.4f | Length: %.4f\n', ...
            epoch, options.epochs, lossHistory.total(epoch), lossHistory.data(epoch), ...
            lossHistory.collision(epoch), lossHistory.smooth(epoch), lossHistory.length(epoch));
    end
end

%% 生成优化后的路径
t_final = linspace(0, 1, numTrainPoints)';
t_final_dl = dlarray(t_final', 'CB');

% 前向传播获取优化路径
path_optimized_dl = forward(net, t_final_dl);
path_optimized = extractdata(path_optimized_dl)';

% 强制边界条件 (确保起点终点精确匹配)
path_optimized(1, :) = source;
path_optimized(end, :) = goal;

% 碰撞后处理：对靠近障碍物的点进行投影修正
path_optimized = collisionPostProcess(path_optimized, circleCenter, r, options.safeMargin);

optimizedPath = path_optimized;

%% 可视化对比
if options.verbose
    figure('Name', 'PINN路径优化对比', 'Position', [100 100 1400 500]);
    
    % 原始路径
    subplot(1, 3, 1);
    plot3(originalPath(:,1), originalPath(:,2), originalPath(:,3), 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
    hold on;
    plotObstacles(circleCenter, r);
    plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title('原始RRT+APF路径');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; grid on;
    
    % 优化后路径
    subplot(1, 3, 2);
    plot3(optimizedPath(:,1), optimizedPath(:,2), optimizedPath(:,3), 'r-o', 'LineWidth', 1.5, 'MarkerSize', 4);
    hold on;
    plotObstacles(circleCenter, r);
    plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title('PINN优化后路径');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; grid on;
    
    % 损失曲线
    subplot(1, 3, 3);
    semilogy(lossHistory.total, 'k-', 'LineWidth', 2); hold on;
    semilogy(lossHistory.data, 'b-', 'LineWidth', 1);
    semilogy(lossHistory.collision, 'r-', 'LineWidth', 1);
    semilogy(lossHistory.smooth, 'g-', 'LineWidth', 1);
    semilogy(lossHistory.length, 'm-', 'LineWidth', 1);
    legend('Total', 'Data', 'Collision', 'Smooth', 'Length', 'Location', 'best');
    xlabel('Epoch'); ylabel('Loss');
    title('PINN训练损失曲线');
    grid on;
    
    % 输出性能指标
    originalLength = computePathLength(originalPath);
    optimizedLength = computePathLength(optimizedPath);
    originalSmooth = computePathSmoothness(originalPath);
    optimizedSmooth = computePathSmoothness(optimizedPath);
    
    fprintf('\n========== PINN优化结果 ==========\n');
    fprintf('路径长度: %.4f -> %.4f (减少 %.2f%%)\n', ...
        originalLength, optimizedLength, (1-optimizedLength/originalLength)*100);
    fprintf('路径平滑度: %.4f -> %.4f (改善 %.2f%%)\n', ...
        originalSmooth, optimizedSmooth, (1-optimizedSmooth/originalSmooth)*100);
    fprintf('==================================\n');
end

% 返回lossHistory供分析
varargout{1} = lossHistory;

end

%% ========== 辅助函数 ==========

function t = computePathParameter(path)
% 计算累积弧长参数
diffs = diff(path);
distances = sqrt(sum(diffs.^2, 2));
t = [0; cumsum(distances)];
t = t / t(end);  % 归一化到[0,1]
end

function totalLength = computePathLength(path)
% 计算路径总长度
diffs = diff(path);
distances = sqrt(sum(diffs.^2, 2));
totalLength = sum(distances);
end

function smoothness = computePathSmoothness(path)
% 计算路径平滑度 (二阶差分的L2范数)
if size(path, 1) < 3
    smoothness = 0;
    return;
end
second_diff = diff(path, 2);
smoothness = sum(sqrt(sum(second_diff.^2, 2)));
end

function plotObstacles(circleCenter, r)
% 绘制障碍物
[x, y, z] = sphere(20);
for i = 1:size(circleCenter, 1)
    mesh(r(i)*x + circleCenter(i,1), r(i)*y + circleCenter(i,2), ...
        r(i)*z + circleCenter(i,3), 'FaceAlpha', 0.3, 'EdgeColor', 'k');
end
end

function path = collisionPostProcess(path, circleCenter, r, safeMargin)
% 碰撞后处理：将靠近障碍物的点沿法向推离
for i = 2:size(path, 1)-1  % 保持起点终点不变
    pos = path(i, :);
    
    for j = 1:size(circleCenter, 1)
        obsPos = circleCenter(j, :);
        dist = norm(pos - obsPos);
        minSafeDist = r(j) + safeMargin;
        
        if dist < minSafeDist
            % 需要修正：沿远离障碍物方向移动
            direction = (pos - obsPos) / dist;
            newPos = obsPos + direction * minSafeDist;
            % 平滑过渡，不完全跳到新位置
            path(i, :) = pos + 0.7 * (newPos - pos);
        end
    end
end
end

%% ========== PINN损失函数 ==========

function [loss, gradients, lossComponents] = modelLoss(net, t_dl, path_target, ...
    circleCenter, r, source, goal, options)

% 前向传播
path_pred = forward(net, t_dl);  % [3 x numPoints]

% 提取数据
numPoints = size(path_pred, 2);
t_val = extractdata(t_dl)';

% 1. 数据拟合损失 (MSE)
loss_data = mse(path_pred, path_target);

% 2. 碰撞避免损失 (物理约束)
% 计算每个预测点到障碍物的惩罚
loss_collision = dlarray(0);
for i = 1:size(circleCenter, 1)
    obsPos = circleCenter(i, :)';
    obsRadius = r(i);
    safeDist = obsRadius + options.safeMargin;
    
    % 计算点到障碍物中心的距离
    diff_vec = path_pred - obsPos;
    distances = sqrt(sum(diff_vec.^2, 1));  % [1 x numPoints]
    
    % 惩罚项：距离小于安全距离时产生惩罚
    penalty = max(0, safeDist - distances);
    loss_collision = loss_collision + mean(penalty.^2);
end

% 3. 平滑性损失 (曲率惩罚)
% 使用有限差分近似计算速度/加速度
if numPoints >= 3
    dt = 1 / (numPoints - 1);
    % 一阶导数 (速度)
    velocity = (path_pred(:, 3:end) - path_pred(:, 1:end-2)) / (2*dt);
    % 二阶导数 (加速度)
    acceleration = (path_pred(:, 3:end) - 2*path_pred(:, 2:end-1) + path_pred(:, 1:end-2)) / (dt^2);
    loss_smooth = mean(sum(acceleration.^2, 1));
else
    loss_smooth = dlarray(0);
end

% 4. 路径长度损失 (鼓励直线化)
if numPoints >= 2
    segments = path_pred(:, 2:end) - path_pred(:, 1:end-1);
    segment_lengths = sqrt(sum(segments.^2, 1));
    loss_length = mean(segment_lengths);
else
    loss_length = dlarray(0);
end

% 5. 边界条件损失 (起点终点固定)
path_val = extractdata(path_pred)';
start_pred = path_val(1, :)';  % 第一个点，转为列向量
end_pred = path_val(end, :)';  % 最后一个点，转为列向量
% 确保source和goal都是列向量
source_col = source(:);
goal_col = goal(:);
loss_boundary = sum((start_pred - source_col).^2) + sum((end_pred - goal_col).^2);
loss_boundary = dlarray(loss_boundary);

% 总损失 (加权组合)
loss = loss_data + ...
       options.lambda_collision * loss_collision + ...
       options.lambda_smooth * loss_smooth + ...
       options.lambda_length * loss_length + ...
       options.lambda_boundary * loss_boundary;

% 返回各组件损失值
lossComponents = struct();
lossComponents.data = double(loss_data);
lossComponents.collision = double(loss_collision);
lossComponents.smooth = double(loss_smooth);
lossComponents.length = double(loss_length);
lossComponents.boundary = double(loss_boundary);

% 计算梯度
gradients = dlgradient(loss, net.Learnables);

end
