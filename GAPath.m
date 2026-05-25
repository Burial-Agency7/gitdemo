%% GAPath.m
% 基于遗传算法的机械臂路径规划
% 输入:
%   source - 起点坐标 [x, y, z]
%   goal - 终点坐标 [x, y, z]
%   circleCenter - 障碍物中心坐标 Nx3
%   r - 障碍物半径 Nx1
%   options - GA参数配置结构体
%   enableLinkCollision - 是否启用连杆碰撞检测
%   PickingRoboticArm - 机器人模型
%   a2, a3, d2 - DH参数
% 输出:
%   path - 规划得到的路径点 Nx3
%   fitnessHistory - 最优适应度变化历史

function [path, fitnessHistory] = GAPath(source, goal, circleCenter, r, options, ...
    enableLinkCollision, PickingRoboticArm, a2, a3, d2)

    %% 参数设置
    if nargin < 5 || isempty(options)
        options = struct();
    end
    
    popSize = getOption(options, 'populationSize', 80);
    maxGen = getOption(options, 'maxGenerations', 200);
    pc = getOption(options, 'crossoverRate', 0.85);
    pm = getOption(options, 'mutationRate', 0.15);
    eliteCount = getOption(options, 'eliteCount', 5);
    numWaypoints = getOption(options, 'waypoints', 15);
    verbose = getOption(options, 'verbose', true);
    pruneEnabled = getOption(options, 'prunePath', true);  % 是否启用路径简化
    
    %% 初始化机器人模型
    % 计算搜索空间边界
    bboxMin = min(source, goal) - 150;
    bboxMax = max(source, goal) + 150;
    
    % 初始化种群: 每个个体是一条路径(中间点的序列)
    population = cell(popSize, 1);
    for i = 1:popSize
        population{i} = initRandomPath(source, goal, numWaypoints, bboxMin, bboxMax);
    end
    
    % 适应度历史
    fitnessHistory = zeros(maxGen, 1);
    
    % 创建图形窗口
    if verbose
        h = figure('Name', 'GA路径规划进化过程', 'Position', [100 100 1200 500]);
    end
    
    %% 主循环
    for gen = 1:maxGen
        % 计算适应度
        fitness = zeros(popSize, 1);
        for i = 1:popSize
            fitness(i) = evaluateFitness(population{i}, source, goal, circleCenter, r, ...
                enableLinkCollision, PickingRoboticArm, a2, a3, d2);
        end
        
        % 记录最优适应度 (注意: 适应度越小越好)
        [bestFitness, bestIdx] = min(fitness);
        fitnessHistory(gen) = bestFitness;
        bestIndividual = population{bestIdx};
        
        % 显示进度
        if verbose && mod(gen, 20) == 0
            fprintf('Generation %d: Best Fitness = %.4f\n', gen, bestFitness);
        end
        
        % 可视化
        if verbose && mod(gen, 10) == 0
            visualizeGA(h, population, bestIndividual, source, goal, circleCenter, r, gen, bestFitness);
        end
        
        % 检查收敛
        if bestFitness < 100 && gen > 50
            if std(fitnessHistory(max(1,gen-20):gen)) < 0.1
                if verbose
                    fprintf('GA收敛于第 %d 代\n', gen);
                end
                break;
            end
        end
        
        %% 选择操作 (锦标赛选择)
        selected = tournamentSelection(population, fitness, popSize);
        
        %% 交叉操作
        offspring = cell(popSize, 1);
        idx = 1;
        while idx <= popSize
            if idx + 1 <= popSize && rand < pc
                [child1, child2] = crossover(selected{idx}, selected{idx+1}, source, goal);
                offspring{idx} = child1;
                if idx + 1 <= popSize
                    offspring{idx+1} = child2;
                end
                idx = idx + 2;
            else
                offspring{idx} = selected{idx};
                idx = idx + 1;
            end
        end
        
        %% 变异操作
        for i = 1:popSize
            if rand < pm
                offspring{i} = mutate(offspring{i}, source, goal, bboxMin, bboxMax, circleCenter, r);
            end
        end
        
        %% 精英保留
        [~, sortedIdx] = sort(fitness);
        elites = population(sortedIdx(1:eliteCount));
        
        % 替换最差个体为精英
        offspringFit = zeros(popSize, 1);
        for i = 1:popSize
            offspringFit(i) = evaluateFitness(offspring{i}, source, goal, circleCenter, r, ...
                enableLinkCollision, PickingRoboticArm, a2, a3, d2);
        end
        [~, worstIdx] = sort(offspringFit, 'descend');
        
        for i = 1:eliteCount
            offspring{worstIdx(i)} = elites{i};
        end
        
        % 更新种群
        population = offspring;
    end
    
    %% 返回最优路径
    % 重新计算最终适应度
    for i = 1:popSize
        fitness(i) = evaluateFitness(population{i}, source, goal, circleCenter, r, ...
            enableLinkCollision, PickingRoboticArm, a2, a3, d2);
    end
    [~, bestIdx] = min(fitness);
    path = population{bestIdx};
    
    %% 路径优化: 移除冗余点 (可选)
    if pruneEnabled
        path = prunePath(path, circleCenter, r);
        % 确保至少有10个点用于后续处理
        if size(path, 1) < 10
            fprintf('警告: 简化后路径点数过少(%d)，重新插值到10点\n', size(path, 1));
            path = interpolatePath(path, 10);
        end
    end
    
    if verbose
        figure(h);
        clf;
        subplot(1, 2, 1);
        plot3(path(:,1), path(:,2), path(:,3), 'r-o', 'LineWidth', 2);
        hold on;
        for i = 1:size(circleCenter, 1)
            [x, y, z] = sphere(20);
            mesh(r(i)*x+circleCenter(i,1), r(i)*y+circleCenter(i,2), r(i)*z+circleCenter(i,3), 'FaceAlpha', 0.3);
        end
        plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        title(sprintf('GA最终路径 (长度=%.2f)', computePathLength(path)));
        xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal; grid on;
        
        subplot(1, 2, 2);
        plot(1:gen, fitnessHistory(1:gen), 'b-', 'LineWidth', 2);
        xlabel('Generation');
        ylabel('Best Fitness');
        title('适应度进化曲线');
        grid on;
    end
end

%% ========== 辅助函数 ==========

function val = getOption(options, field, default)
    if isfield(options, field)
        val = options.(field);
    else
        val = default;
    end
end

function path = initRandomPath(source, goal, numWaypoints, bboxMin, bboxMax)
    % 初始化随机路径: 起点 + 随机中间点 + 终点
    path = zeros(numWaypoints + 2, 3);
    path(1, :) = source;
    path(end, :) = goal;
    
    % 在中间生成随机点
    for i = 2:numWaypoints+1
        t = (i-2) / numWaypoints;
        % 基础位置在起点终点连线上
        basePos = (1-t) * source + t * goal;
        % 添加随机偏移
        offset = (rand(1, 3) - 0.5) * 100;
        path(i, :) = basePos + offset;
        % 限制在边界内
        path(i, :) = max(path(i, :), bboxMin);
        path(i, :) = min(path(i, :), bboxMax);
    end
end

function fitness = evaluateFitness(path, source, goal, circleCenter, r, ...
    enableLinkCollision, PickingRoboticArm, a2, a3, d2)
    
    fitness = 0;
    
    % 1. 路径长度惩罚 (主要目标)
    pathLength = computePathLength(path);
    fitness = fitness + pathLength;
    
    % 2. 碰撞惩罚 (硬性约束)
    collisionPenalty = 0;
    for i = 1:size(path, 1)
        pos = path(i, :);
        for j = 1:size(circleCenter, 1)
            dist = norm(pos - circleCenter(j, :));
            if dist < r(j)
                collisionPenalty = collisionPenalty + 1000 + (r(j) - dist) * 100;
            elseif dist < r(j) + 20
                collisionPenalty = collisionPenalty + (r(j) + 20 - dist) * 10;
            end
        end
    end
    fitness = fitness + collisionPenalty;
    
    % 3. 连杆碰撞检测 (如果启用) - 分层采样策略
    if enableLinkCollision
        linkCollisionPenalty = evaluateLinkCollisionFast(path, circleCenter, r, PickingRoboticArm, a2, a3, d2);
        fitness = fitness + linkCollisionPenalty;
    end
    
    % 4. 路径平滑度惩罚
    if size(path, 1) >= 3
        second_diff = diff(path, 2);
        smoothPenalty = sum(sqrt(sum(second_diff.^2, 2))) * 5;
        fitness = fitness + smoothPenalty;
    end
    
    % 5. 起点终点固定检查
    if norm(path(1, :) - source) > 1
        fitness = fitness + 1000;
    end
    if norm(path(end, :) - goal) > 1
        fitness = fitness + 1000;
    end
end

function selected = tournamentSelection(population, fitness, popSize)
    selected = cell(popSize, 1);
    tournamentSize = 3;
    
    for i = 1:popSize
        % 随机选择tournamentSize个个体
        idx = randperm(length(population), tournamentSize);
        tournamentFit = fitness(idx);
        [~, bestIdx] = min(tournamentFit);
        selected{i} = population{idx(bestIdx)};
    end
end

function [child1, child2] = crossover(parent1, parent2, source, goal)
    % 部分匹配交叉 (PMX) 的变体
    n = size(parent1, 1);
    
    % 保持起点终点不变，交叉中间部分
    child1 = parent1;
    child2 = parent2;
    
    % 随机选择交叉点
    cp1 = randi([2, n-2]);
    cp2 = randi([cp1+1, n-1]);
    
    % 交换中间段
    child1(cp1:cp2, :) = parent2(cp1:cp2, :);
    child2(cp1:cp2, :) = parent1(cp1:cp2, :);
    
    % 确保起点终点不变
    child1(1, :) = source;
    child1(end, :) = goal;
    child2(1, :) = source;
    child2(end, :) = goal;
end

function mutated = mutate(individual, source, goal, bboxMin, bboxMax, circleCenter, r)
    mutated = individual;
    n = size(individual, 1);
    
    % 随机选择一个中间点进行变异
    idx = randi([2, n-1]);
    
    % 生成新位置: 当前位置 + 随机偏移
    currentPos = individual(idx, :);
    
    % 计算前后点的方向，进行引导变异
    prevPos = individual(idx-1, :);
    nextPos = individual(idx+1, :);
    
    % 在前后点连线附近随机移动
    midPoint = (prevPos + nextPos) / 2;
    offset = (rand(1, 3) - 0.5) * 60;
    newPos = midPoint + offset;
    
    % 确保不碰撞
    for j = 1:size(circleCenter, 1)
        dist = norm(newPos - circleCenter(j, :));
        if dist < r(j) + 30
            % 向远离障碍物的方向移动
            direction = (newPos - circleCenter(j, :)) / (dist + 1e-6);
            newPos = circleCenter(j, :) + direction * (r(j) + 35);
        end
    end
    
    % 限制在边界内
    newPos = max(newPos, bboxMin);
    newPos = min(newPos, bboxMax);
    
    mutated(idx, :) = newPos;
    
    % 确保起点终点不变
    mutated(1, :) = source;
    mutated(end, :) = goal;
end

function path = prunePath(path, circleCenter, r)
    % 移除冗余路径点: 如果两点之间可以直接连接而不碰撞，则移除中间点
    if size(path, 1) <= 2
        return;
    end
    
    pruned = path(1, :);
    i = 1;
    while i < size(path, 1)
        % 从后向前找可以直接连接的点
        found = false;
        for j = size(path, 1):-1:i+1
            if isPathFree(path(i, :), path(j, :), circleCenter, r, 10)
                pruned = [pruned; path(j, :)];
                i = j;
                found = true;
                break;
            end
        end
        if ~found
            i = i + 1;
            if i <= size(path, 1)
                pruned = [pruned; path(i, :)];
            end
        end
    end
    path = pruned;
end

function free = isPathFree(p1, p2, circleCenter, r, numChecks)
    free = true;
    for t = linspace(0, 1, numChecks)
        pos = (1-t) * p1 + t * p2;
        for j = 1:size(circleCenter, 1)
            if norm(pos - circleCenter(j, :)) < r(j) + 5
                free = false;
                return;
            end
        end
    end
end

function newPath = interpolatePath(path, nPoints)
    % 对路径进行线性插值，确保有足够的点数
    if size(path, 1) < 2
        newPath = path;
        return;
    end
    t = linspace(0, 1, size(path, 1));
    tNew = linspace(0, 1, nPoints);
    newPath = zeros(nPoints, 3);
    for i = 1:3
        newPath(:, i) = interp1(t, path(:, i), tNew, 'linear');
    end
end

%% ========== 高效连杆碰撞检测函数 ==========

function penalty = evaluateLinkCollisionFast(path, circleCenter, r, PickingRoboticArm, a2, a3, d2)
    % 极致优化版 - 单点检测+启发式预筛选
    penalty = 0;
    n = size(path, 1);
    
    % 步骤1：找到唯一最关键点（路径中靠近障碍物的极值点）
    minDist = inf;
    criticalIdx = round(n/2);
    criticalObsIdx = 1;
    
    % 极稀疏采样找全局最近点（每10个点采样1个）
    for i = 1:10:n
        for j = 1:size(circleCenter, 1)
            d = norm(path(i,:) - circleCenter(j,:)) - r(j);
            if d < minDist
                minDist = d;
                criticalIdx = i;
                criticalObsIdx = j;
            end
        end
    end
    
    % 步骤2：启发式预筛选 - 基于末端位置快速判断连杆碰撞可能性
    % 如果末端在某些安全区域，连杆几乎不可能碰撞
    endPos = path(criticalIdx, :);
    obsPos = circleCenter(criticalObsIdx, :);
    
    % 快速几何启发式：如果末端在障碍物上方或外侧足够远，跳过大臂检测
    dx = endPos(1) - obsPos(1);
    dy = endPos(2) - obsPos(2);
    dz = endPos(3) - obsPos(3);
    horizontalDist = sqrt(dx^2 + dy^2);
    
    % 安全区域启发式：如果末端水平距离远或垂直距离高，大概率安全
    armLength = a2 + a3 + 50;  % 大臂+小臂+裕度
    if horizontalDist > r(criticalObsIdx) + armLength || abs(dz) > 200
        % 快速通过，无碰撞风险
        return;
    end
    
    % 步骤3：仅对高风险点进行完整检测（单点！）
    T = eye(4);
    T(1:3, 4) = endPos';
    T(1:3, 1:3) = [1 0 0; 0 -1 0; 0 0 1];
    
    try
        q = PickingRoboticArm.ikunc(T);
        % 只检测单障碍物（之前找到的最接近的）
        if checkSingleObstacleCollision(q, circleCenter(criticalObsIdx, :), r(criticalObsIdx), a2, a3, d2)
            penalty = penalty + 500;
        end
    catch
        penalty = penalty + 50;
    end
end

%% 单障碍物大臂碰撞检测 - 极速版（无循环）
function collision = checkSingleObstacleCollision(q, obsCenter, obsR, a2, a3, d2)
    % 无循环，单障碍物检测
    collision = false;
    d1 = 41;
    
    % 快速估算肩部位置（三角函数）
    shoulderZ = d1 + d2 * sin(q(2));
    shoulderXY = d2 * cos(q(2));
    
    % 快速估算肘部位置
    sumAngle = q(2) + q(3);
    elbowZ = shoulderZ + a2 * sin(sumAngle);
    elbowXY = shoulderXY + a2 * cos(sumAngle);
    
    % 计算连杆线段
    shoulder = [shoulderXY * cos(q(1)), shoulderXY * sin(q(1)), shoulderZ];
    elbow = [elbowXY * cos(q(1)), elbowXY * sin(q(1)), elbowZ];
    
    % 点到线段距离计算（内联展开，无函数调用开销）
    ab = elbow - shoulder;
    ap = obsCenter - shoulder;
    ab_dot_ab = dot(ab, ab);
    
    if ab_dot_ab < 1e-10
        dist = norm(obsCenter - shoulder);
    else
        t = max(0, min(1, dot(ap, ab) / ab_dot_ab));
        closest = shoulder + t * ab;
        dist = norm(obsCenter - closest);
    end
    
    % 单障碍物判断
    threshold = obsR + 45;  % 连杆半径+裕度
    if dist < threshold
        collision = true;
    end
end

function totalLength = computePathLength(path)
    diffs = diff(path);
    distances = sqrt(sum(diffs.^2, 2));
    totalLength = sum(distances);
end

function visualizeGA(h, population, best, source, goal, circleCenter, r, gen, bestFit)
    figure(h);
    clf;
    
    subplot(1, 2, 1);
    hold on;
    
    % 绘制所有路径(透明)
    for i = 1:min(20, length(population))
        p = population{i};
        plot3(p(:,1), p(:,2), p(:,3), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    end
    
    % 绘制最优路径
    plot3(best(:,1), best(:,2), best(:,3), 'r-o', 'LineWidth', 2);
    
    % 绘制障碍物
    for i = 1:size(circleCenter, 1)
        [x, y, z] = sphere(20);
        mesh(r(i)*x+circleCenter(i,1), r(i)*y+circleCenter(i,2), r(i)*z+circleCenter(i,3), 'FaceAlpha', 0.3);
    end
    
    plot3(source(1), source(2), source(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal(1), goal(2), goal(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    title(sprintf('Generation %d, Best Fitness: %.2f', gen, bestFit));
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; grid on;
    
    drawnow limitrate;
end
