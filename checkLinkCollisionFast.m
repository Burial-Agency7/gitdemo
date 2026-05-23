%% checkLinkCollisionFast.m
% 快速连杆碰撞检测 - 只检查关键连杆(大臂、小臂)以提升性能
% 输入:
%   q - 6x1关节角度向量
%   circleCenter - Nx3障碍物球心坐标
%   r - Nx1障碍物半径
%   a2, a3, d2 - 机器人DH参数
% 输出:
%   collision - true表示发生碰撞, false表示无碰撞
%   collisionInfo - 碰撞信息结构体
function [collision, collisionInfo] = checkLinkCollisionFast(q, circleCenter, r, a2, a3, d2)
    collision = false;
    collisionInfo = struct('linkIdx', [], 'obsIdx', [], 'distance', []);
    
    % 获取各关节位置
    jointPos = getJointPositions(q, a2, a3, d2);
    
    % 简化连杆半径
    linkRadius = 40;
    safetyMargin = 8;
    
    % 只检查关键连杆: 2(大臂根部), 3(大臂末端), 4(小臂)
    criticalLinks = [2, 3, 4];
    
    for linkIdx = criticalLinks
        p1 = jointPos(linkIdx, :);
        p2 = jointPos(linkIdx+1, :);
        
        for obsIdx = 1:size(circleCenter, 1)
            center = circleCenter(obsIdx, :);
            radius = r(obsIdx) + linkRadius + safetyMargin;
            
            % 快速距离检查
            [dist, ~] = pointToSegmentDistanceFast(center, p1, p2);
            
            if dist < radius
                collision = true;
                collisionInfo.linkIdx = linkIdx;
                collisionInfo.obsIdx = obsIdx;
                collisionInfo.distance = dist;
                return;
            end
        end
    end
end

%% 快速点到线段距离计算
function [dist, closestPoint] = pointToSegmentDistanceFast(p, a, b)
    ab = b - a;
    ap = p - a;
    ab_len_sq = sum(ab.^2);
    
    if ab_len_sq < 1e-10
        closestPoint = a;
        dist = norm(p - a);
        return;
    end
    
    t = max(0, min(1, sum(ap .* ab) / ab_len_sq));
    closestPoint = a + t * ab;
    dist = norm(p - closestPoint);
end
