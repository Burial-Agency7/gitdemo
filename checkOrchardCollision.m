function [collision, collisionInfo] = checkOrchardCollision(q, env, PickingRoboticArm, a2, a3, d2, fastMode)
%% 果园环境连杆碰撞检测
% 支持圆柱形（树干）和球形（树冠/果实）障碍物
% 输入:
%   q - 6x1关节角度
%   env - 果园环境结构体
%   PickingRoboticArm - 机器人模型
%   fastMode - true: 快速模式(仅末端), false: 完整检测
% 输出:
%   collision - true/false
%   collisionInfo - 详细信息结构体

if nargin < 6
    fastMode = false;
end

collision = false;
collisionInfo = [];

%% 获取机器人各连杆端点位置
try
    % 正运动学计算各关节位置
    T_all = PickingRoboticArm.fkine(q);
    
    % 提取各连杆端点
    jointPos = zeros(7, 3);  % 基座+6个关节
    jointPos(1, :) = [0, 0, 0];  % 基座原点
    
    for i = 1:6
        Ti = PickingRoboticArm.A(1:i, q);
        T_curr = Ti.T;
        jointPos(i+1, :) = T_curr(1:3, 4)';
    end
    
    % 定义连杆线段 (从基座到末端)
    % 连杆1: 基座到关节2
    % 连杆2: 关节2到关节3 (大臂)
    % 连杆3: 关节3到关节4 (小臂)
    % 连杆4-6: 腕部连杆
    linkSegments = {
        jointPos(1:2, :),   % 连杆1
        jointPos(2:3, :),   % 连杆2 (大臂)
        jointPos(3:4, :),   % 连杆3 (小臂)
        jointPos(4:5, :),   % 连杆4
        jointPos(5:6, :),   % 连杆5
        jointPos(6:7, :)    % 连杆6 (末端)
    };
    
    % 快速模式：只检测末端位置
    if fastMode
        endEffectorPos = jointPos(7, :);
        for i = 1:size(env.obstacles, 1)
            obsPos = env.obstacles(i, :);
            obsType = env.obsType(i);
            obsR = env.obsRadius(i);
            margin = getOrchardMargin(obsType, env);
            
            dist = norm(endEffectorPos - obsPos);
            if dist < (obsR + margin)
                collision = true;
                collisionInfo.linkIdx = 6;
                collisionInfo.obsIdx = i;
                collisionInfo.obsType = obsType;
                collisionInfo.distance = dist;
                return;
            end
        end
        return;
    end
    
    % 完整模式：逐连杆检测
    for linkIdx = 1:length(linkSegments)
        segment = linkSegments{linkIdx};
        p1 = segment(1, :);
        p2 = segment(2, :);
        
        % 对每个障碍物检测
        for obsIdx = 1:size(env.obstacles, 1)
            obsPos = env.obstacles(obsIdx, :);
            obsType = env.obsType(obsIdx);
            obsR = env.obsRadius(obsIdx);
            obsH = env.obsHeight(obsIdx);
            margin = getOrchardMargin(obsType, env);
            
            % 根据障碍物类型选择检测方法
            if obsType == 1 || obsType == 3  % 圆柱形（树干/机器人）
                cylCollision = checkCylinderLineCollision(p1, p2, obsPos, obsR, obsH);
                if cylCollision
                    collision = true;
                    collisionInfo.linkIdx = linkIdx;
                    collisionInfo.obsIdx = obsIdx;
                    collisionInfo.obsType = obsType;
                    collisionInfo.linkName = getLinkName(linkIdx);
                    collisionInfo.obsName = env.typeLabels{obsType};
                    return;
                end
            else  % 球形/椭球形（树冠/果实）
                % 简化为球形检测：线段到球心最近距离
                [closestPoint, t] = pointLineSegmentClosest(obsPos, p1, p2);
                if t >= 0 && t <= 1
                    dist = norm(closestPoint - obsPos);
                    if dist < (obsR + margin)
                        collision = true;
                        collisionInfo.linkIdx = linkIdx;
                        collisionInfo.obsIdx = obsIdx;
                        collisionInfo.obsType = obsType;
                        collisionInfo.linkName = getLinkName(linkIdx);
                        collisionInfo.obsName = env.typeLabels{obsType};
                        return;
                    end
                end
            end
        end
    end
    
catch ME
    % 出错时保守地认为碰撞
    warning('碰撞检测出错: %s', ME.message);
    collision = true;
    collisionInfo.error = ME.message;
end

end

%% 辅助函数：圆柱-线段碰撞检测
function collision = checkCylinderLineCollision(p1, p2, cylCenter, cylRadius, cylHeight)
    % 圆柱轴线（垂直）
    cylBottom = cylCenter - [0, 0, cylHeight/2];
    cylTop = cylCenter + [0, 0, cylHeight/2];
    
    % 计算线段到圆柱轴线的最近距离
    [dist, t, s] = lineSegmentToLineSegmentDistance(p1, p2, cylBottom, cylTop);
    
    % 检查是否在圆柱半径范围内
    if dist > cylRadius
        collision = false;
        return;
    end
    
    % 检查z方向是否在圆柱高度范围内
    % 线段上的最近点
    if t < 0
        closestOnSegment = p1;
    elseif t > 1
        closestOnSegment = p2;
    else
        closestOnSegment = p1 + t * (p2 - p1);
    end
    
    zMin = cylCenter(3) - cylHeight/2;
    zMax = cylCenter(3) + cylHeight/2;
    
    % 如果线段穿过圆柱的垂直范围
    if closestOnSegment(3) >= zMin && closestOnSegment(3) <= zMax
        collision = true;
        return;
    end
    
    % 检查端点是否在圆柱内（距离判断）
    for point = {p1, p2}
        pt = point{1};
        distXY = norm(pt(1:2) - cylCenter(1:2));  % 水平距离
        if distXY < cylRadius && pt(3) >= zMin && pt(3) <= zMax
            collision = true;
            return;
        end
    end
    
    collision = false;
end

function [dist, t, s] = lineSegmentToLineSegmentDistance(p1, p2, q1, q2)
    % 计算两条线段之间的最短距离及参数
    u = p2 - p1;
    v = q2 - q1;
    w = p1 - q1;
    
    a = dot(u, u);
    b = dot(u, v);
    c = dot(v, v);
    d = dot(u, w);
    e = dot(v, w);
    D = a*c - b*b;
    
    sD = D;
    tD = D;
    SMALL_NUM = 1e-10;
    
    % 计算线段最近点参数
    if D < SMALL_NUM  % 线段近似平行
        sN = 0;
        sD = 1;
        tN = e;
        tD = c;
    else
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if sN < 0
            sN = 0;
            tN = e;
            tD = c;
        elseif sN > sD
            sN = sD;
            tN = e + b;
            tD = c;
        end
    end
    
    if tN < 0
        tN = 0;
        if -d < 0
            sN = 0;
        elseif -d > a
            sN = sD;
        else
            sN = -d;
            sD = a;
        end
    elseif tN > tD
        tN = tD;
        if (-d + b) < 0
            sN = 0;
        elseif (-d + b) > a
            sN = sD;
        else
            sN = (-d + b);
            sD = a;
        end
    end
    
    s = sN / sD;
    t = tN / tD;
    
    pClosest = p1 + s * u;
    qClosest = q1 + t * v;
    dist = norm(pClosest - qClosest);
end

function [closestPoint, t] = pointLineSegmentClosest(point, lineStart, lineEnd)
    % 点到线段的最近点
    v = lineEnd - lineStart;
    w = point - lineStart;
    c1 = dot(w, v);
    if c1 <= 0
        closestPoint = lineStart;
        t = 0;
        return;
    end
    c2 = dot(v, v);
    if c2 <= c1
        closestPoint = lineEnd;
        t = 1;
        return;
    end
    t = c1 / c2;
    closestPoint = lineStart + t * v;
end

function margin = getOrchardMargin(obsType, env)
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

function name = getLinkName(idx)
    names = {'基座连杆', '大臂', '小臂', '腕部连杆1', '腕部连杆2', '末端执行器'};
    if idx <= length(names)
        name = names{idx};
    else
        name = sprintf('连杆%d', idx);
    end
end
