function env = simpleOrchard(variant)
%% 简化版果园环境
% variant: 'minimal'(极简3棵树), 'standard'(标准5棵树), 'sparse'(稀疏7棵树)

if nargin < 1
    variant = 'standard';
end

env.name = ['简化果园 (' variant ')'];

%% 根据版本配置障碍物
switch variant
    case 'minimal'
        % 极简版: 3棵树呈三角形分布 (缩小并移至工作空间内)
        treePos = [150, -80, 100;     % 中间
                   80, -120, 80;      % 左后方
                   80, 80, 80];       % 右后方
        treeRadius = [35; 30; 30];   % 缩小半径(原80/60→35/30)
        treeHeight = [120; 100; 100]; % 降低高度
        
    case 'standard'
        % 标准版: 5棵树形成通道 (限制在工作空间200mm半径内)
        treePos = [100, -120, 80;    % 左侧
                   130, 0, 90;       % 中左
                   100, 120, 80;     % 右侧
                   180, -80, 70;     % 前方左侧
                   180, 80, 70];     % 前方右侧
        treeRadius = [30; 35; 30; 25; 25];  % 统一缩小
        treeHeight = [120; 140; 120; 110; 110];
        
    case 'sparse'
        % 稀疏版: 起点[80,-140,80] 终点[220,0,120]
        % 混合布局: 部分在连线上形成阻挡，部分偏离连线增加干扰
        treePos = [140, -100, 95;    % 起点-终点连线中间偏左(关键阻挡)
                   180, -60, 100;    % 起点-终点连线右侧(关键阻挡)
                   200, 30, 105;     % 终点附近偏右(目标树区域)
                   110, 80, 85;      % 左侧偏离连线(干扰)
                   160, 100, 90;     % 右前方偏离连线(干扰)
                   120, -60, 80;     % 起点附近偏离连线(干扰)
                   220, -80, 100];    % 终点下方偏离连线(额外阻挡)
        treeRadius = [35; 38; 32; 28; 30; 28; 30];  % 大小不一
        treeHeight = [120; 130; 115; 100; 110; 100; 115];
end

%% 统一用球形简化表示(合并树干和树冠)
env.obstacles = treePos;           % 障碍物中心位置
env.obsRadius = treeRadius;        % 等效半径
env.obsHeight = treeHeight;        % 仅用于可视化高度

%% 起点终点设计(确保在工作空间内)
% 机械臂工作空间: 以基座为中心，半径约250mm
% 起点终点范围: X[50, 250], Y[-200, 200], Z[50, 250]
switch variant
    case 'minimal'
        env.source = [60, -60, 80];       % 后方左侧起点
        env.goal = [200, 0, 150];          % 前方中央目标树
        
    case 'standard'
        env.source = [50, -100, 70];       % 左后方入口
        env.goal = [200, 0, 160];          % 前方中央采摘点
        
    case 'sparse'
        % 起点距离基座约180mm，终点距离约230mm(在工作空间内)
        % 连线方向大致沿X轴，穿过中间障碍物
        env.source = [80, -140, 80];       % 左后方起点，距离约178mm
        env.goal = [220, 0, 120];          % 前方目标点，距离约246mm，连线穿过障碍物
end

%% 工作空间边界
margin = 100;
env.bounds.xMin = min([env.source(1), env.goal(1), treePos(:,1)']) - margin;
env.bounds.xMax = max([env.source(1), env.goal(1), treePos(:,1)']) + margin;
env.bounds.yMin = min([env.source(2), env.goal(2), treePos(:,2)']) - margin;
env.bounds.yMax = max([env.source(2), env.goal(2), treePos(:,2)']) + margin;
env.bounds.zMin = 0;
env.bounds.zMax = max(treeHeight) + 150;

%% 安全距离
env.safetyMargin = 20;  % 缩小安全余量(障碍物已大幅缩小)

%% 可视化颜色
env.treeColor = [0.3, 0.6, 0.3];  % 树绿色
env.groundColor = [0.8, 0.85, 0.7]; % 浅黄绿地面

fprintf('简化果园环境 [%s]: %d棵树\n', variant, size(treePos, 1));
fprintf('  起点: [%.0f, %.0f, %.0f]\n', env.source);
fprintf('  终点: [%.0f, %.0f, %.0f]\n', env.goal);

end

%% 简化碰撞检测 (仅球形)
function collision = checkSimpleOrchardCollision(point, env)
    % 点到所有障碍物的距离检测
    collision = false;
    for i = 1:size(env.obstacles, 1)
        dist = norm(point - env.obstacles(i, :));
        if dist < (env.obsRadius(i) + env.safetyMargin)
            collision = true;
            return;
        end
    end
end
