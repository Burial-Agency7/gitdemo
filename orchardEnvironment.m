function env = orchardEnvironment(scenarioType)
%% 果园环境模型配置
% 输入: scenarioType - 'static'(静态),'dynamic'(动态),'mixed'(混合)
% 输出: env结构体包含障碍物、起点终点、边界信息

env.name = '果园作业环境';
env.scenarioType = scenarioType;

%% 果园布局参数 (标准苹果园配置)
rowSpacing = 400;      % 行距 4m = 400mm (实际果园3-5m，此处按比例缩小)
treeSpacing = 250;     % 株距 2.5m = 250mm
treeHeight = 300;      % 树高 3m = 300mm
crownRadius = 120;     % 树冠半径 1.2m = 120mm
trunkRadius = 25;      % 树干半径 0.25m = 25mm
trunkHeight = 80;      % 树干高度 0.8m = 80mm

%% 生成行列排列的果树（静态障碍物）
numRows = 3;           % 行数
numTreesPerRow = 4;    % 每行树数

staticObs = [];
staticType = [];
staticRadius = [];
staticHeight = [];

for row = 1:numRows
    for col = 1:numTreesPerRow
        x = 150 + (row-1) * rowSpacing;
        y = -150 + (col-1) * treeSpacing;
        z = 0;  % 地面
        
        % 树干（圆柱形）
        staticObs = [staticObs; x, y, z + trunkHeight/2];
        staticType = [staticType; 1];  % 1=圆柱
        staticRadius = [staticRadius; trunkRadius];
        staticHeight = [staticHeight; trunkHeight];
        
        % 树冠（椭球形或球形简化）
        staticObs = [staticObs; x, y, z + trunkHeight + crownRadius*0.7];
        staticType = [staticType; 2];  % 2=树冠（球形简化）
        staticRadius = [staticRadius; crownRadius];
        staticHeight = [staticHeight; crownRadius*1.4];  % 椭球高度
    end
end

%% 动态障碍物（根据场景类型）
dynamicObs = [];
dynamicType = [];
dynamicRadius = [];
dynamicHeight = [];
dynamicVelocity = [];

if strcmp(scenarioType, 'dynamic') || strcmp(scenarioType, 'mixed')
    % 移动中的其他机器人/运输车
    dynamicObs = [dynamicObs; 400, 0, 100];  % 位置
    dynamicType = [dynamicType; 3];  % 3=移动机器人（圆柱）
    dynamicRadius = [dynamicRadius; 60];
    dynamicHeight = [dynamicHeight; 150];
    dynamicVelocity = [dynamicVelocity; -20, 10, 0];  % mm/s，向左前方移动
    
    % 掉落的果实（地面小球，低高度）
    dynamicObs = [dynamicObs; 250, 50, 15];
    dynamicType = [dynamicType; 4];  % 4=果实（球）
    dynamicRadius = [dynamicRadius; 15];
    dynamicHeight = [dynamicHeight; 15];
    dynamicVelocity = [dynamicVelocity; 0, 0, 0];  % 静态
end

%% 合并障碍物
env.obstacles = [staticObs; dynamicObs];
env.obsType = [staticType; dynamicType];
env.obsRadius = [staticRadius; dynamicRadius];
env.obsHeight = [staticHeight; dynamicHeight];
env.obsVelocity = [zeros(size(staticObs,1),3); dynamicVelocity];

%% 障碍物类型标签
env.typeLabels = {'树干', '树冠', '移动机器人', '果实'};

%% 起点终点配置（果园作业任务）
% 起点：果园入口/装卸区
env.source = [50, -200, 100];  % 地面附近，果园边缘

% 终点：采摘目标树（第2行第3棵树上方）
targetTreeIdx = 2 * numTreesPerRow + 3;
if targetTreeIdx <= size(staticObs, 1)
    targetPos = staticObs(targetTreeIdx, :);
    env.goal = [targetPos(1), targetPos(2), targetPos(3) + crownRadius + 50];  % 树冠上方
else
    env.goal = [500, 200, 250];  % 默认目标
end

%% 作业空间边界
env.bounds.xMin = 0;
env.bounds.xMax = 800;
env.bounds.yMin = -300;
env.bounds.yMax = 400;
env.bounds.zMin = 0;
env.bounds.zMax = 500;

%% 安全距离配置（机械臂与不同障碍物）
env.safetyMargin.trunk = 50;    % 树干安全余量
env.safetyMargin.crown = 80;    % 树冠安全余量（需更大空间）
env.safetyMargin.robot = 100;   % 其他机器人安全余量
env.safetyMargin.fruit = 20;    % 果实安全余量

%% 可视化参数
env.vis.treeColor = [0.2, 0.6, 0.2];      % 树冠绿色
env.vis.trunkColor = [0.4, 0.2, 0.1];     % 树干棕色
env.vis.robotColor = [0.8, 0.8, 0.2];     % 机器人黄色
env.vis.fruitColor = [0.9, 0.3, 0.1];     % 果实红色

fprintf('果园环境已生成: %d 个静态障碍物, %d 个动态障碍物\n', ...
    size(staticObs,1), size(dynamicObs,1));

end

%% 辅助函数：圆柱-线段碰撞检测
function collision = checkCylinderCollision(p1, p2, cylCenter, cylRadius, cylHeight)
    % 检测线段p1-p2与圆柱的碰撞
    % 圆柱: 底面中心cylCenter, 半径cylRadius, 高度cylHeight
    
    % 圆柱轴线方向（假设垂直）
    axisStart = cylCenter - [0, 0, cylHeight/2];
    axisEnd = cylCenter + [0, 0, cylHeight/2];
    
    % 计算线段到圆柱轴线的最近距离
    [dist, ~] = distanceLineSegmentToLineSegment(p1, p2, axisStart, axisEnd);
    
    % 检查是否在半径和高度范围内
    collision = dist < cylRadius;
    if collision
        % 还需检查z方向是否在圆柱高度范围内
        zMin = cylCenter(3) - cylHeight/2;
        zMax = cylCenter(3) + cylHeight/2;
        segmentZMin = min(p1(3), p2(3));
        segmentZMax = max(p1(3), p2(3));
        collision = ~(segmentZMax < zMin || segmentZMin > zMax);
    end
end

function [dist, t, s] = distanceLineSegmentToLineSegment(p1, p2, q1, q2)
    % 计算两条线段之间的最短距离
    u = p2 - p1;
    v = q2 - q1;
    w = p1 - q1;
    
    a = dot(u, u);
    b = dot(u, v);
    c = dot(v, v);
    d = dot(u, w);
    e = dot(v, w);
    D = a*c - b*b;
    
    if D < 1e-10  % 线段平行
        t = 0;
        s = 0;
    else
        s = (b*e - c*d) / D;
        t = (a*e - b*d) / D;
        s = max(0, min(1, s));
        t = max(0, min(1, t));
    end
    
    pClosest = p1 + s * u;
    qClosest = q1 + t * v;
    dist = norm(pClosest - qClosest);
end

%% 辅助函数：获取安全距离根据障碍物类型
function margin = getSafetyMargin(obsType, env)
    switch obsType
        case 1
            margin = env.safetyMargin.trunk;
        case 2
            margin = env.safetyMargin.crown;
        case 3
            margin = env.safetyMargin.robot;
        case 4
            margin = env.safetyMargin.fruit;
        otherwise
            margin = 50;
    end
end
