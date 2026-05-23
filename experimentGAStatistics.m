function stats = experimentGAStatistics(numTrials)
% EXPERIMENTGASTATISTICS GA+PINN算法实验数据统计
%
% 输入:
%   numTrials - 每种场景重复试验次数，默认3
%
% 输出:
%   stats - 结构体包含所有实验数据
%
% 使用示例:
%   stats = experimentGAStatistics();      % 默认3次试验
%   stats = experimentGAStatistics(5);   % 5次试验
%   displayGAResults(stats);

%% 默认参数
if nargin < 1
    numTrials = 10;  % 每个场景3次试验
end

fprintf('========================================\n');
fprintf('   GA+PINN 实验数据统计\n');
fprintf('========================================\n\n');

% 场景配置
scenarios = {'simple', 'medium', 'complex'};
scenarioNames = {'简单场景', '中等场景', '复杂密集场景'};

% 初始化统计数据
stats = struct();
for s = 1:length(scenarios)
    stats(s).scenario = scenarioNames{s};
    stats(s).collisionRate = zeros(numTrials, 1);
    stats(s).planTime = zeros(numTrials, 1);
    stats(s).pathLength = zeros(numTrials, 1);
    stats(s).smoothness = zeros(numTrials, 1);
    stats(s).positionError = zeros(numTrials, 1);
end

%% 运行实验
for s = 1:length(scenarios)
    fprintf('【场景 %d/%d】%s\n', s, length(scenarios), scenarioNames{s});
    
    for trial = 1:numTrials
        fprintf('  试验 %d/%d... ', trial, numTrials);
        
        % 运行单次实验
        result = runSingleGAExperiment(scenarios{s});
        
        % 记录数据
        stats(s).collisionRate(trial) = result.collisionRate;
        stats(s).planTime(trial) = result.planTime;
        stats(s).pathLength(trial) = result.pathLength;
        stats(s).smoothness(trial) = result.smoothness;
        stats(s).positionError(trial) = result.positionError;
        
        fprintf('完成 (%.2fs)\n', result.planTime);
    end
    
    % 计算统计值
    stats(s).avgPlanTime = mean(stats(s).planTime);
    stats(s).stdPlanTime = std(stats(s).planTime);
    stats(s).avgPathLength = mean(stats(s).pathLength);
    stats(s).pathLengthVar = std(stats(s).pathLength) / mean(stats(s).pathLength) * 100;
    stats(s).avgSmoothness = mean(stats(s).smoothness);
    stats(s).stdSmoothness = std(stats(s).smoothness);
    stats(s).avgPositionError = mean(stats(s).positionError);
    stats(s).collisionCount = sum(stats(s).collisionRate);
    
    fprintf('  平均规划时间: %.2f±%.2f s\n', stats(s).avgPlanTime, stats(s).stdPlanTime);
    fprintf('  平均路径长度: %.1f mm (变异系数: %.1f%%)\n', stats(s).avgPathLength, stats(s).pathLengthVar);
    fprintf('  平均平滑度: %.2f±%.2f\n', stats(s).avgSmoothness, stats(s).stdSmoothness);
    fprintf('  碰撞次数: %d/%d\n\n', stats(s).collisionCount, numTrials);
end

%% 显示汇总表
fprintf('========================================\n');
fprintf('         GA+PINN 实验结果汇总\n');
fprintf('========================================\n');
displayGAResults(stats, scenarioNames, numTrials);

%% 保存结果
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('GA_experiment_stats_%s.mat', timestamp);
save(filename, 'stats');
fprintf('\n数据已保存: %s\n', filename);

end

%% ========== 运行单次实验 ==========
function result = runSingleGAExperiment(scenarioType)
    % 配置GA参数
    gaOptions = struct();
    gaOptions.populationSize = 80;
    gaOptions.maxGenerations = 1000;
    gaOptions.crossoverRate = 0.85;
    gaOptions.mutationRate = 0.15;
    gaOptions.waypoints = 15;
    gaOptions.prunePath = false;
    gaOptions.verbose = false;
    
    % 创建场景
    env = createScenario(scenarioType);
    
    % 初始化机器人模型
    a2 = 120; a3 = 136; d2 = 20;
    d1 = 41;
    L1 = Link('d', d1, 'a', 0, 'alpha', 0, 'modified');
    L2 = Link('d', d2, 'a', 0, 'alpha', pi/2, 'modified');
    L3 = Link('d', -20, 'a', a2, 'alpha', 0, 'modified');
    L4 = Link('d', 0, 'a', a3, 'alpha', pi/2, 'modified');
    L5 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'modified');
    L6 = Link('d', 50, 'a', 0, 'alpha', 0, 'modified');
    PickingRoboticArm = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'PickingRoboticArm');
    
    % 运行GA规划
    tic;
    [path, ~] = GAPath(env.source, env.goal, env.obstacles, env.obsRadius, ...
        gaOptions, false, PickingRoboticArm, a2, a3, d2);
    gaTime = toc;
    
    % 运行PINN优化
    pinnOptions = createPINNOptions();
    optimizedPath = pinnOptimizePath(path, env.obstacles, env.obsRadius, ...
        env.source, env.goal, pinnOptions);
    pinnTime = toc - gaTime;
    
    % 确保路径是double类型且有效
    if isa(optimizedPath, 'dlarray')
        optimizedPath = extractdata(optimizedPath);
    end
    optimizedPath = double(optimizedPath);
    
    % 检查路径是否有效
    if isempty(optimizedPath) || any(isnan(optimizedPath(:)))
        warning('PINN优化返回无效路径，使用原始路径');
        optimizedPath = path;
    end
    
    % 计算指标
    result.planTime = gaTime + pinnTime;
    result.pathLength = computePathLength(optimizedPath);
    result.smoothness = computePathSmoothness(optimizedPath);
    result.positionError = norm(optimizedPath(end,:) - env.goal);
    result.collisionRate = checkPathCollision(optimizedPath, env.obstacles, env.obsRadius);
end

%% ========== 场景创建 ==========
function env = createScenario(type)
    env = struct();
    switch type
        case 'simple'
            env.source = [50, 50, 100];
            env.goal = [200, 200, 150];
            env.obstacles = [120, 120, 80; 150, 100, 100];
            env.obsRadius = [30; 25];
        case 'medium'
            env.source = [50, 50, 100];
            env.goal = [250, 250, 150];
            theta = linspace(0, pi, 5)';
            env.obstacles = [150 + 50*cos(theta), 150 + 50*sin(theta), 100*ones(5,1)];
            env.obsRadius = 30 * ones(5, 1);
        case 'complex'
            env.source = [30, 30, 80];
            env.goal = [280, 280, 180];
            [X, Y] = meshgrid(80:40:240, 80:40:240);
            env.obstacles = [X(:), Y(:), 100*ones(length(X(:)),1)];
            env.obsRadius = 25 * ones(length(X(:)), 1);
    end
end

%% ========== PINN参数 ==========
function opt = createPINNOptions()
    opt = struct();
    opt.hiddenLayers = [32, 32, 16];  % 进一步简化网络
    opt.epochs = 300;  % 减少训练轮数
    opt.learningRate = 0.0005;  % 降低学习率
    opt.lambda_collision = 10;
    opt.lambda_smooth = 50;
    opt.lambda_length = 2;
    opt.lambda_boundary = 100;
    opt.numPoints = 100;
    opt.safeMargin = 5;
    opt.verbose = false;
end

%% ========== 计算指标 ==========
function L = computePathLength(path)
    if size(path, 1) < 2
        L = 0;  % 空路径或单点路径，长度为0
        return;
    end
    L = sum(sqrt(sum(diff(path).^2, 2)));
end

function S = computePathSmoothness(path)
    if size(path, 1) < 3
        S = 0;
        return;
    end
    S = sum(sqrt(sum(diff(path, 2).^2, 2)));
end

function collision = checkPathCollision(path, obstacles, r)
    collision = 0;
    for i = 1:size(path, 1)
        for j = 1:size(obstacles, 1)
            if norm(path(i,:) - obstacles(j,:)) < r(j)
                collision = 1;
                return;
            end
        end
    end
end

%% ========== 显示结果 ==========
function displayGAResults(stats, names, numTrials)
    fprintf('\n【GA+PINN 详细数据表】\n');
    fprintf('%-12s %-10s %-12s %-15s %-12s %-12s %-12s\n', ...
        '场景', '碰撞率(%)', '规划时间(s)', '路径长度(mm)', '变异系数(%)', '平滑度', '定位误差(mm)');
    fprintf('%s\n', repmat('-', 1, 95));
    
    for s = 1:length(stats)
        collisionRate = stats(s).collisionCount / numTrials * 100;
        fprintf('%-12s %-10.1f %-12.2f %-15.1f %-12.1f %-12.2f %-12.2f\n', ...
            names{s}, collisionRate, stats(s).avgPlanTime, stats(s).avgPathLength, ...
            stats(s).pathLengthVar, stats(s).avgSmoothness, stats(s).avgPositionError);
    end
    
    % 综合统计
    fprintf('\n【综合性能】\n');
    avgTime = mean([stats.avgPlanTime]);
    avgLen = mean([stats.avgPathLength]);
    avgVar = mean([stats.pathLengthVar]);
    avgSmooth = mean([stats.avgSmoothness]);
    totalCollision = sum([stats.collisionCount]);
    
    fprintf('平均规划时间: %.2f s\n', avgTime);
    fprintf('平均路径长度: %.1f mm\n', avgLen);
    fprintf('平均变异系数: %.1f %%\n', avgVar);
    fprintf('平均平滑度: %.2f\n', avgSmooth);
    fprintf('总碰撞率: %.1f%% (%d/%d)\n', totalCollision/(numTrials*length(stats))*100, totalCollision, numTrials*length(stats));
end
