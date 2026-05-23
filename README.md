# GA遗传算法 + 连杆碰撞检测 + PINN路径优化

本文件夹包含基于遗传算法(GA)的机械臂避障路径规划代码，并集成了PINN(物理信息神经网络)进行路径优化。

## 文件说明

| 文件名 | 功能描述 |
|--------|----------|
| `mainga_pinn.m` | 主程序，配置参数并执行完整的路径规划流程 |
| `GAPath.m` | 遗传算法路径规划核心函数 |
| `pinnOptimizePathClassic.m` | PINN经典优化方法(使用fmincon) |
| `checkLinkCollision.m` | 完整连杆碰撞检测 |
| `checkLinkCollisionFast.m` | 快速连杆碰撞检测(只检测关键连杆) |
| `getJointPositions.m` | 根据关节角度计算各关节位置(正运动学) |
| `threespline.m` | 三次B样条曲线平滑 |

## 使用方法

### 1. 运行主程序

在MATLAB中运行:
```matlab
mainga_pinn
```

### 2. 配置参数

在 `mainga_pinn.m` 文件中可以修改以下关键参数:

#### 连杆碰撞检测开关
```matlab
enableLinkCollision = true;  % true=启用, false=禁用
```
- `true`: 检测机械臂各连杆与障碍物的碰撞(计算量大但更安全)
- `false`: 只检测末端执行器与障碍物的碰撞(计算快但可能漏检)

#### PINN优化开关
```matlab
usePINN = true;  % true=启用PINN优化, false=仅使用GA+B样条
```
- `true`: GA路径规划后使用PINN进行优化，然后再B样条平滑
- `false`: GA路径规划后直接进行B样条平滑

### 3. 机器人参数

机械臂DH参数(改进DH法):
- `a2 = 120`: 连杆2长度
- `a3 = 136`: 连杆3长度
- `d2 = 20`: 连杆2偏置

### 4. 路径配置

```matlab
source = [120 -180 80];  % 起点坐标
goal = [200, 180, 180];   % 终点坐标
```

### 5. 障碍物配置

```matlab
circleCenterSave = [200, -100, 80;  % 障碍物1中心
                  180, 0, 120;      % 障碍物2中心
                  150, 160, 260];   % 障碍物3中心
rSave = [45; 45; 45];              % 障碍物半径
```

## GA算法参数配置

在 `mainga_pinn.m` 中可调整GA参数:

```matlab
gaOptions.populationSize = 80;      % 种群大小
gaOptions.maxGenerations = 200;     % 最大迭代次数
gaOptions.crossoverRate = 0.85;     % 交叉概率
gaOptions.mutationRate = 0.15;      % 变异概率
gaOptions.eliteCount = 5;          % 精英保留数量
gaOptions.waypoints = 15;           % 路径中间点数
```

## 程序流程

```
[配置参数]
    ↓
[GA遗传算法路径规划] → 生成初始避障路径
    ↓
[连杆碰撞检测验证] (可选)
    ↓
[PINN路径优化] (可选，由usePINN控制)
    ↓
[B样条曲线平滑]
    ↓
[连杆碰撞检测验证] (可选)
    ↓
[机器人运动学计算与动画可视化]
    ↓
[性能评估与对比]
```

## 输出结果

程序运行后会生成以下图形:

1. **Figure 1**: GA进化过程动画
2. **Figure 2**: B样条平滑后的路径
3. **Figure 4**: 机器人执行路径动画(带末端轨迹)
4. **Figure 5**: 关节空间轨迹图
5. **路径对比图**: 原始路径 vs PINN优化后 vs 最终路径

## 性能指标

程序会自动计算并输出以下性能指标:
- 路径长度
- 路径平滑度(越小越好)
- 终点定位误差
- 各阶段计算耗时

## 依赖工具箱

- Robotics Toolbox (Peter Corke)
- Optimization Toolbox (用于fmincon)

## 与RRT+APF参考代码的对应关系

| 本代码 | RRT参考代码 |
|--------|-------------|
| `mainga_pinn.m` | `mainrrt_pinn.m` |
| `GAPath.m` | `RTTPath_APF.m` |
| `pinnOptimizePathClassic.m` | `pinnOptimizePathClassic.m` |
| `checkLinkCollision.m` | `checkLinkCollision.m` |
| `threespline.m` | `threespline.m` |

机械臂参数、障碍物设置、可视化效果与参考代码保持一致。
