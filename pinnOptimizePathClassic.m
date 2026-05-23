function optimizedPath = pinnOptimizePathClassic(originalPath, circleCenter, r, source, goal, options)
%PINNOPTIMIZEPATHCLASSIC 经典优化方法实现PINN路径优化
% 使用带物理约束的非线性优化替代神经网络
% 优点: 无需Deep Learning Toolbox，计算更快，结果可解释性强
%
% 输入输出同 pinnOptimizePath.m

%% 默认参数设置
if nargin < 6
    options = struct();
end
if ~isfield(options, 'maxIter'), options.maxIter = 500; end
if ~isfield(options, 'numPoints'), options.numPoints = 80; end
if ~isfield(options, 'safeMargin'), options.safeMargin = 5; end
if ~isfield(options, 'lambda_collision'), options.lambda_collision = 100; end
if ~isfield(options, 'lambda_smooth'), options.lambda_smooth = 50; end
if ~isfield(options, 'lambda_length'), options.lambda_length = 10; end
if ~isfield(options, 'verbose'), options.verbose = true; end

%% 路径参数化与初始化
N = size(originalPath, 1);
if N < 3
    optimizedPath = originalPath;
    return;
end

% 使用直线初始化
t_opt = linspace(0, 1, options.numPoints)';
path_init = (1 - t_opt) .* source + t_opt .* goal;

% 根据原始路径调整初始点
if size(originalPath, 1) >= 3
    for i = 2:options.numPoints-1
        linePoint = (1 - t_opt(i)) * source + t_opt(i) * goal;
        
        minDistToObs = inf;
        for j = 1:size(circleCenter, 1)
            dist = norm(linePoint - circleCenter(j, :));
            minDistToObs = min(minDistToObs, dist - r(j));
        end
        
        if minDistToObs < options.safeMargin + 10
            [~, nearestIdx] = min(sqrt(sum((originalPath - linePoint).^2, 2)));
            offset = originalPath(nearestIdx, :) - linePoint;
            path_init(i, :) = linePoint + 0.7 * offset;
        end
    end
end

% 强制边界条件
path_init(1, :) = source;
path_init(end, :) = goal;

%% 使用fmincon进行带约束优化
x0 = reshape(path_init(2:end-1, :)', [], 1);

bboxMin = min(source, goal) - 100;
bboxMax = max(source, goal) + 100;
numVars = length(x0);
lb = repmat(bboxMin', numVars/3, 1);
ub = repmat(bboxMax', numVars/3, 1);

optOptions = optimoptions('fmincon', ...
    'Algorithm', 'interior-point', ...
    'MaxIterations', options.maxIter, ...
    'MaxFunctionEvaluations', options.maxIter * 10, ...
    'OptimalityTolerance', 1e-6, ...
    'StepTolerance', 1e-8, ...
    'Display', 'iter', ...
    'PlotFcn', []);

if ~options.verbose
    optOptions.Display = 'off';
end

collisionConstraint = @(x) makeCollisionConstraint(x, source, goal, circleCenter, r, options.safeMargin);

lambda_straight = 20;
objective = @(x) pinnObjectiveV2(x, source, goal, originalPath, t_opt, ...
    options.lambda_collision, options.lambda_smooth, options.lambda_length, lambda_straight, circleCenter, r);

%% 执行优化
if options.verbose
    disp('开始经典PINN优化 (fmincon)...');
end

tic;
[x_opt, ~, ~] = fmincon(objective, x0, [], [], [], [], lb, ub, collisionConstraint, optOptions);
optTime = toc;

% 重构优化后的路径
innerPoints = reshape(x_opt, 3, [])';
optimizedPath = [source; innerPoints; goal];

%% 碰撞后处理
optimizedPath = collisionPostProcess(optimizedPath, circleCenter, r, options.safeMargin);

%% 可视化
if options.verbose
    originalLength = computePathLength(originalPath);
    optimizedLength = computePathLength(optimizedPath);
    originalSmooth = computePathSmoothness(originalPath);
    optimizedSmooth = computePathSmoothness(optimizedPath);
    
    figure('Name', '经典PINN优化结果', 'Position', [100 100 1200 400]);
    
    subplot(1, 2, 1);
    plot3(originalPath(:,1), originalPath(:,2), originalPath(:,3), 'b-o', 'LineWidth', 1.5);
    hold on;
    plotObstacles(circleCenter, r);
    plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title(sprintf('原始路径\n长度=%.2f, 平滑度=%.2f', originalLength, originalSmooth));
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; grid on;
    
    subplot(1, 2, 2);
    plot3(optimizedPath(:,1), optimizedPath(:,2), optimizedPath(:,3), 'r-o', 'LineWidth', 1.5);
    hold on;
    plotObstacles(circleCenter, r);
    plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title(sprintf('PINN优化后\n长度=%.2f, 平滑度=%.2f\n耗时=%.2fs', optimizedLength, optimizedSmooth, optTime));
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; grid on;
    
    fprintf('\n========== 经典PINN优化结果 ==========\n');
    fprintf('优化耗时: %.3f 秒\n', optTime);
    fprintf('路径长度: %.4f -> %.4f (减少 %.2f%%)\n', ...
        originalLength, optimizedLength, (1-optimizedLength/originalLength)*100);
    fprintf('路径平滑度: %.4f -> %.4f (改善 %.2f%%)\n', ...
        originalSmooth, optimizedSmooth, (1-optimizedSmooth/originalSmooth)*100);
    fprintf('======================================\n');
end

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

function plotObstacles(circleCenter, r)
    [x, y, z] = sphere(20);
    for i = 1:size(circleCenter, 1)
        mesh(r(i)*x + circleCenter(i,1), r(i)*y + circleCenter(i,2), ...
            r(i)*z + circleCenter(i,3), 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    end
end

function path = collisionPostProcess(path, circleCenter, r, safeMargin)
    for i = 2:size(path, 1)-1
        pos = path(i, :);
        for j = 1:size(circleCenter, 1)
            obsPos = circleCenter(j, :);
            dist = norm(pos - obsPos);
            minSafeDist = r(j) + safeMargin;
            if dist < minSafeDist
                direction = (pos - obsPos) / dist;
                newPos = obsPos + direction * minSafeDist;
                path(i, :) = pos + 0.7 * (newPos - pos);
            end
        end
    end
end

%% ========== 目标函数 V2 ==========

function loss = pinnObjectiveV2(x, source, goal, originalPath, t_opt, ...
    lambda_collision, lambda_smooth, lambda_length, lambda_straight, circleCenter, r)
    
    numInner = length(x) / 3;
    innerPoints = reshape(x, 3, numInner)';
    path = [source; innerPoints; goal];

    N = size(path, 1);
    loss = 0;

    % 1. 直线化约束
    straightLineVec = goal - source;
    straightLineVec = straightLineVec / norm(straightLineVec);
    straightLoss = 0;
    for i = 2:N-1
        toPoint = path(i, :) - source;
        projection = dot(toPoint, straightLineVec);
        closestOnLine = source + projection * straightLineVec;
        deviation = norm(path(i, :) - closestOnLine);
        straightLoss = straightLoss + deviation^2;
    end
    loss = loss + lambda_straight * straightLoss;

    % 2. 碰撞避免项
    collisionLoss = 0;
    for i = 2:N-1
        pos = path(i, :);
        for j = 1:size(circleCenter, 1)
            dist = norm(pos - circleCenter(j, :));
            safeDist = r(j) + 15;
            if dist < safeDist
                collisionLoss = collisionLoss + (safeDist - dist)^2;
            end
        end
    end
    loss = loss + lambda_collision * collisionLoss;

    % 3. 平滑性项
    if N >= 3
        second_diff = diff(path, 2);
        smoothLoss = sum(sum(second_diff.^2, 2));
        loss = loss + lambda_smooth * smoothLoss;
    end

    % 4. 路径长度项
    if N >= 2
        segments = diff(path);
        segmentLengths = sqrt(sum(segments.^2, 2));
        meanSegLen = mean(segmentLengths);
        lengthLoss = sum((segmentLengths - meanSegLen).^2);
        loss = loss + lambda_length * lengthLoss;
    end

    % 5. 弱数据拟合项
    path_interp = interp1(t_opt, path, linspace(0, 1, max(3, size(originalPath, 1)))', 'linear', 'extrap');
    sampleIndices = round(linspace(1, size(originalPath, 1), size(path_interp, 1)))';
    sampledOriginal = originalPath(sampleIndices, :);
    dataFitLoss = sum(sqrt(sum((path_interp - sampledOriginal).^2, 2))) / size(path_interp, 1);
    loss = loss + 0.1 * dataFitLoss;
end

%% ========== 非线性约束函数 ==========

function [c, ceq] = makeCollisionConstraint(x, source, goal, circleCenter, r, safeMargin)
    numInner = length(x) / 3;
    innerPoints = reshape(x, 3, numInner)';
    path = [source; innerPoints; goal];

    numObstacles = size(circleCenter, 1);
    numPoints = size(path, 1);

    c = zeros(numPoints * numObstacles, 1);
    idx = 1;

    for i = 1:numPoints
        for j = 1:numObstacles
            dist = norm(path(i, :) - circleCenter(j, :));
            c(idx) = (r(j) + safeMargin) - dist;
            idx = idx + 1;
        end
    end

    ceq = [];
end
