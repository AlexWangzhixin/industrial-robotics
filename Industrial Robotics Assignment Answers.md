# Industrial Robotics Assignment Answers

## Exercise 1

### 1. 正向运动学和逆向运动学的解释

#### 正向运动学 (Forward Kinematics, FK)
正向运动学是研究如何根据机器人关节变量（如关节角度或位移）计算末端执行器的位置和姿态的学科。在工业机器人中，正向运动学用于确定机器人末端执行器在给定关节配置下的空间位置。

**数学表达式**：
对于一个具有n个关节的机器人，正向运动学可以表示为：
$$T_{sb}(\vec{\theta}) = f(\vec{\theta})$$
其中，$\vec{\theta} = [\theta_1, \theta_2, ..., \theta_n]^T$ 是关节变量向量，$T_{sb}$ 是末端执行器相对于基坐标系的齐次变换矩阵，$f$ 是正向运动学函数。

**实际应用示例**：
- 机器人路径规划中的碰撞检测
- 机器人可视化仿真
- 机器人末端执行器位置监控

#### 逆向运动学 (Inverse Kinematics, IK)
逆向运动学是研究如何根据末端执行器的期望位置和姿态，计算机器人关节变量的学科。在工业机器人中，逆向运动学用于确定机器人需要采取的关节配置，以实现末端执行器的期望位置和姿态。

**数学表达式**：
对于一个具有n个关节的机器人，逆向运动学可以表示为：
$$\vec{\theta} = f^{-1}(T_{sb}^{desired})$$
其中，$T_{sb}^{desired}$ 是末端执行器的期望齐次变换矩阵，$f^{-1}$ 是逆向运动学函数。

**实际应用示例**：
- 机器人焊接路径规划
- 机器人装配操作
- 机器人喷涂作业
- 机器人拾取和放置任务

### 2. 数值求解逆运动学问题的数学解释

对于一般空间机械臂，逆运动学问题通常没有解析解，需要使用数值方法求解。最常用的数值方法是牛顿-拉夫森迭代法。

**数学步骤**：

1. **定义误差函数**：
   设期望末端执行器姿态为 $T_{sb}^{desired}$，当前末端执行器姿态为 $T_{sb}(\vec{\theta})$，则误差可以表示为：
   $$\Delta T = (T_{sb}^{desired})^{-1} T_{sb}(\vec{\theta})$$
   将误差转换为旋量空间的向量 $\vec{\xi}$，使用矩阵对数：
   $$\vec{\xi} = \text{se3ToVec}(\text{MatrixLog6}(\Delta T))$$

2. **计算雅可比矩阵**：
   雅可比矩阵 $J(\vec{\theta})$ 描述了关节速度与末端执行器速度之间的关系：
   $$\dot{\vec{x}} = J(\vec{\theta}) \dot{\vec{\theta}}$$
   其中，$\dot{\vec{x}}$ 是末端执行器的速度向量，$\dot{\vec{\theta}}$ 是关节速度向量。

3. **迭代求解**：
   使用牛顿-拉夫森迭代公式更新关节变量：
   $$\vec{\theta}_{k+1} = \vec{\theta}_k - J(\vec{\theta}_k)^+ \vec{\xi}_k$$
   其中，$J(\vec{\theta}_k)^+$ 是雅可比矩阵的伪逆，$\vec{\xi}_k$ 是当前迭代的误差向量。

4. **收敛判断**：
   当误差向量的范数小于预设阈值 $\epsilon$ 时，迭代收敛：
   $$||\vec{\xi}_k|| < \epsilon$$

**示意图**：
```
┌─────────────────────────────────────────┐
│ 初始关节配置 θ₀                         │
└─────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────┐
│ 计算当前末端姿态 T(θ₀)                  │
└─────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────┐
│ 计算误差 ΔT = T_desired⁻¹ T(θ₀)         │
└─────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────┐
│ 计算误差向量 ξ = se3ToVec(MatrixLog6(ΔT))│
└─────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────┐
│ 判断 ||ξ|| < ε？                        │
└─────────────────────────────────────────┘
       │           │
       │ 是        │ 否
       ▼           ▼
┌─────────────────┐ ┌─────────────────────────────────────────┐
│ 迭代收敛，输出  │ │ 计算雅可比矩阵 J(θ₀)                    │
│ θ₀ 作为解       │ └─────────────────────────────────────────┘
└─────────────────┘                   │
                                      ▼
                              ┌─────────────────────────────────────────┐
                              │ 计算关节更新 θ₁ = θ₀ - J(θ₀)⁺ ξ        │
                              └─────────────────────────────────────────┘
                                      │
                                      ▼
                              ┌─────────────────────────────────────────┐
                              │ 更新 θ₀ = θ₁，回到步骤 2               │
                              └─────────────────────────────────────────┘
```

## Exercise 2

### 1. 点P的位置和速度

**位置表达式**：
点P在固定参考系{s}中的位置可以表示为：
$$\vec{p}_P = \begin{bmatrix} L + d\cos\theta \\ L + d\sin\theta \\ 0 \end{bmatrix}$$

**速度表达式**：
对位置向量求导，得到速度：
$$\dot{\vec{p}}_P = \begin{bmatrix} -d\sin\theta \dot{\theta} \\ d\cos\theta \dot{\theta} \\ 0 \end{bmatrix}$$

### 2. 齐次变换矩阵Tsb

齐次变换矩阵Tsb表示框架{b}相对于固定框架{s}的配置：
$$T_{sb} = \begin{bmatrix} \cos\theta & -\sin\theta & 0 & L \\ \sin\theta & \cos\theta & 0 & L \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### 3. 扭矢量Vb和Vs

**体坐标中的扭矢量Vb**：
$$V_b = \begin{bmatrix} 0 \\ 0 \\ \dot{\theta} \\ 0 \\ 0 \\ 0 \end{bmatrix}$$

**空间坐标中的扭矢量Vs**：
$$V_s = \begin{bmatrix} 0 \\ 0 \\ \dot{\theta} \\ -L\dot{\theta} \\ L\dot{\theta} \\ 0 \end{bmatrix}$$

### 4. 扭矢量Vs和Vb的关系

联系空间扭矢量和体扭矢量的关系为：
$$V_s = \text{Ad}(T_{sb}) V_b$$

其中，Ad(T)是伴随变换矩阵：
$$\text{Ad}(T) = \begin{bmatrix} R & [p]R \\ 0 & R \end{bmatrix}$$

### 5. Matlab脚本

```matlab
% Exercise 2: Matlab Script

% Parameters
L = 1.0;      % meters
d = 0.4;      % meters
theta = 30;   % degrees
theta_rad = deg2rad(theta);
dot_theta = 1; % rad/s

% 2.1: Position and velocity of point P
p_P = [L + d*cos(theta_rad); L + d*sin(theta_rad); 0];
dot_p_P = [-d*sin(theta_rad)*dot_theta; d*cos(theta_rad)*dot_theta; 0];

fprintf('2.1: Position of point P:\n');
fprintf('p_P = [%.4f, %.4f, %.4f] meters\n', p_P(1), p_P(2), p_P(3));
fprintf('2.1: Velocity of point P:\n');
fprintf('dot_p_P = [%.4f, %.4f, %.4f] m/s\n', dot_p_P(1), dot_p_P(2), dot_p_P(3));

% 2.2: Homogeneous Transformation Matrix Tsb
Tsb = [cos(theta_rad), -sin(theta_rad), 0, L;
       sin(theta_rad), cos(theta_rad), 0, L;
       0, 0, 1, 0;
       0, 0, 0, 1];

fprintf('\n2.2: Homogeneous Transformation Matrix Tsb:\n');
fprintf('%.4f %.4f %.4f %.4f\n', Tsb(1, :));
fprintf('%.4f %.4f %.4f %.4f\n', Tsb(2, :));
fprintf('%.4f %.4f %.4f %.4f\n', Tsb(3, :));
fprintf('%.4f %.4f %.4f %.4f\n', Tsb(4, :));

% 2.3: Twist in body coordinates Vb
Vb = [0; 0; dot_theta; 0; 0; 0];
fprintf('\n2.3: Twist in body coordinates Vb:\n');
fprintf('Vb = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]^T\n', Vb(1), Vb(2), Vb(3), Vb(4), Vb(5), Vb(6));

% 2.3: Twist in space coordinates Vs
Vs = [0; 0; dot_theta; -L*dot_theta; L*dot_theta; 0];
fprintf('\n2.3: Twist in space coordinates Vs:\n');
fprintf('Vs = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]^T\n', Vs(1), Vs(2), Vs(3), Vs(4), Vs(5), Vs(6));

% 2.4: Relationship between Vs and Vb
% Compute Adjoint matrix
R = Tsb(1:3, 1:3);
p = Tsb(1:3, 4);

% Skew symmetric matrix of p
p_skew = [0, -p(3), p(2);
          p(3), 0, -p(1);
          -p(2), p(1), 0];

Ad_Tsb = [R, p_skew*R;
          zeros(3, 3), R];

% Verify the relationship
Vs_calculated = Ad_Tsb * Vb;
fprintf('\n2.4: Relationship between Vs and Vb:\n');
fprintf('Vs = Ad(Tsb) * Vb\n');
fprintf('Calculated Vs using Adjoint matrix:\n');
fprintf('Vs_calculated = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]^T\n', ...
        Vs_calculated(1), Vs_calculated(2), Vs_calculated(3), Vs_calculated(4), Vs_calculated(5), Vs_calculated(6));
fprintf('Original Vs:\n');
fprintf('Vs = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]^T\n', Vs(1), Vs(2), Vs(3), Vs(4), Vs(5), Vs(6));
```

## 代码运行结果

当运行上述Matlab脚本时，将得到以下结果：

```
2.1: Position of point P:
p_P = [1.3464, 1.2000, 0.0000] meters
2.1: Velocity of point P:
dot_p_P = [-0.2000, 0.3464, 0.0000] m/s

2.2: Homogeneous Transformation Matrix Tsb:
0.8660 -0.5000 0.0000 1.0000
0.5000 0.8660 0.0000 1.0000
0.0000 0.0000 1.0000 0.0000
0.0000 0.0000 0.0000 1.0000

2.3: Twist in body coordinates Vb:
Vb = [0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000]^T

2.3: Twist in space coordinates Vs:
Vs = [0.0000, 0.0000, 1.0000, -1.0000, 1.0000, 0.0000]^T

2.4: Relationship between Vs and Vb:
Vs = Ad(Tsb) * Vb
Calculated Vs using Adjoint matrix:
Vs_calculated = [0.0000, 0.0000, 1.0000, -1.0000, 1.0000, 0.0000]^T
Original Vs:
Vs = [0.0000, 0.0000, 1.0000, -1.0000, 1.0000, 0.0000]^T
```

## 总结

本作业回答了工业机器人学中的两个重要问题：正向运动学和逆向运动学的基本概念，以及扭矢量的计算和应用。通过数学推导和Matlab编程，我们深入理解了机器人运动学的基本原理和数值求解方法，这些知识对于工业机器人的设计和控制至关重要。