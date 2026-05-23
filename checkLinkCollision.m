%% checkLinkCollision.m
% 检查机器人连杆与球形障碍物的碰撞
% 输入:
%   q - 6x1关节角度向量
%   circleCenter - Nx3障碍物球心坐标
%   r - Nx1障碍物半径
%   a2, a3, d2 - 机器人DH参数
% 输出:
%   collision - true表示发生碰撞, false表示无碰撞
%   collisionInfo - 碰撞信息结构体
function [collision, collisionInfo] = checkLinkCollision(q, circleCenter, r, a2, a3, d2)
    collision = false;
    collisionInfo = struct('linkIdx', [], 'obsIdx', [], 'distance', []);
    
    % 获取各关节位置
    jointPos = getJointPositions(q, a2, a3, d2);
    
    % 定义连杆半径 (用于包裹连杆的圆柱体半径)
    linkRadius = 45;
    safetyMargin = 10;
    
    % 检查6个连杆 (关节i到关节i+1)
    for linkIdx = 1:6
        p1 = jointPos(linkIdx, :);
        p2 = jointPos(linkIdx+1, :);
        
        % 检查与每个障碍物的碰撞
        for obsIdx = 1:size(circleCenter, 1)
            center = circleCenter(obsIdx, :);
            radius = r(obsIdx) + linkRadius + safetyMargin;
            
            % 计算线段到球心的最短距离
            [dist, ~] = pointToSegmentDistance(center, p1, p2);
            
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

%% 计算点到线段的最短距离
function [dist, closestPoint] = pointToSegmentDistance(p, a, b)
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
