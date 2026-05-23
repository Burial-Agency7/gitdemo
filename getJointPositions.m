%% getJointPositions.m
% 根据关节角度计算各关节在基坐标系中的位置
% 基于改进DH参数法
function jointPositions = getJointPositions(q, a2, a3, d2)
    % DH参数 (改进DH法)
    % L1: d=41, a=0, alpha=0
    % L2: d=d2, a=0, alpha=pi/2
    % L3: d=-20, a=a2, alpha=0
    % L4: d=0, a=a3, alpha=pi/2
    % L5: d=0, a=0, alpha=pi/2
    % L6: d=50, a=0, alpha=0
    
    d1 = 41;
    d3 = -20;
    d6 = 50;
    
    % 初始化关节位置矩阵 (7个关节位置: 基座+6个关节)
    jointPositions = zeros(7, 3);
    
    % 基座位置
    jointPositions(1, :) = [0, 0, 0];
    
    % 计算各变换矩阵和关节位置
    T = eye(4);
    
    % 关节1
    T1 = dhTransformModified(q(1), d1, 0, 0);
    T = T * T1;
    jointPositions(2, :) = T(1:3, 4)';
    
    % 关节2
    T2 = dhTransformModified(q(2), d2, 0, pi/2);
    T = T * T2;
    jointPositions(3, :) = T(1:3, 4)';
    
    % 关节3
    T3 = dhTransformModified(q(3), d3, a2, 0);
    T = T * T3;
    jointPositions(4, :) = T(1:3, 4)';
    
    % 关节4
    T4 = dhTransformModified(q(4), 0, a3, pi/2);
    T = T * T4;
    jointPositions(5, :) = T(1:3, 4)';
    
    % 关节5
    T5 = dhTransformModified(q(5), 0, 0, pi/2);
    T = T * T5;
    jointPositions(6, :) = T(1:3, 4)';
    
    % 关节6 (末端)
    T6 = dhTransformModified(q(6), d6, 0, 0);
    T = T * T6;
    jointPositions(7, :) = T(1:3, 4)';
end

%% 改进DH变换矩阵
function T = dhTransformModified(theta, d, a, alpha)
    T = [cos(theta), -sin(theta), 0, a;
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha);
         0, 0, 0, 1];
end
