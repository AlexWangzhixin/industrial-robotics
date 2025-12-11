# Industrial Robotics Assignment Instructions

## 作业要求

### Exercise 1: Kinematics Theory and Numerical Methods

1. **理论解释**：用文字、示意图和数学公式解释什么是正运动学 (Forward Kinematics) 和逆运动学 (Inverse Kinematics)，并通过具体例子说明它们在工业机器人中的应用。

2. **数值解法原理**：给出清晰的数学解释，说明如何通过数值方法求解具有多个自由度的一般空间机械臂的逆运动学问题，提供迭代求解过程中涉及的详细数学步骤。

3. **编程实现**：针对图4所示的冗余机械臂，编写一个Matlab逆运动学求解器（不要使用Matlab库中预制的函数，如IKinSpace），并通过测试不同的配置来证明求解器工作正常。

4. **奇异性分析**：找到逆运动学求解器无法找到\(\vec{\theta}\)解的配置，并解释原因。

### Exercise 2: Planar Rigid Body Motion

参考图1所示的示意图：图中右上角的物体绕坐标为\((L, L)\)的点旋转，角速度为\(\dot{\theta} = 1 \text{ rad/s}\)。手算解决以下问题，并详细解释每一步推导和计算过程：

1. **位置与速度**：将动坐标系\(\{b\}\)上点\(P\)的位置\(\vec{p}_P\)用固定参考系\(\{s\}\)表示（变量为\(\theta\)）；然后求出点\(P\)相对于固定系\(\{s\}\)的速度\(\vec{\dot{p}}_P\)。

2. **齐次变换矩阵**：写出表示从固定系\(\{s\}\)看坐标系\(\{b\}\)的齐次变换矩阵\(T_{sb}\)。

3. **旋量**：写出\(T_{sb}\)的旋量表达式，分别在物体坐标系下表示为\(\mathcal{V}_b\)，以及在空间坐标系下表示为\(\mathcal{V}_s\)。

4. **旋量关系**：表达出\(\mathcal{V}_s\)和\(\mathcal{V}_b\)之间的关系。

5. **Matlab编程**：编写一个Matlab脚本，当\(L=1.0\text{ m}\)，\(d=0.4\text{ m}\)且\(\theta=30^\circ\)时，求解上述2.1到2.4的所有问题。

### Exercise 3: Inverse Kinematics using Spatial Jacobian

针对SCARA机器人（如图5所示），实现基于空间雅可比矩阵的逆运动学求解器，并完成以下任务：

1. **理论解释**：用文字和数学公式解释空间雅可比矩阵的定义、计算方法以及如何用于逆运动学求解。

2. **编程实现**：编写一个Matlab脚本，实现基于空间雅可比矩阵的逆运动学求解器，使用牛顿-拉夫森迭代法，不使用Matlab库中预制的函数（如IKinSpace）。

3. **测试与验证**：
   - 使用不同的初始猜测值测试求解器
   - 验证求解结果的正确性
   - 分析求解器的收敛性和鲁棒性

4. **轨迹规划**：利用求解器规划一条从初始位姿到目标位姿的平滑轨迹，使用三次时间缩放函数生成关节空间轨迹。

## 代码运行说明

### 环境要求

- Matlab R2018a或更高版本
- Windows操作系统

### 代码结构

```
FC（2）-251029工业机器人PGEE11212
├── Code/
│   ├── IKinBodyIterative.m      # Exercise 1: 逆运动学求解器
│   ├── Exercise2_Solver.m        # Exercise 2: 平面刚体运动求解器
│   └── IKinSpaceIterative.m      # Exercise 3: 基于空间雅可比的逆运动学求解器
├── 编程库/
│   └── Industrial_Robotics_Library/  # 工业机器人函数库
├── Industrial_Robotics_Assignment_English.pdf  # 作业解答PDF
└── Assignment_Instructions.md    # 本文档
```

### 运行步骤

#### 1. Exercise 1: 逆运动学求解器

**功能说明**：实现了基于牛顿-拉夫森迭代法的逆运动学求解器，使用雅可比矩阵伪逆来求解关节角度。

**使用方法**：

1. 打开Matlab，将工作目录设置为`Code`文件夹。
2. 在Matlab命令窗口中调用`IKinBodyIterative`函数，输入以下参数：
   ```matlab
   Blist = [...];      % 物体坐标系下的螺旋轴
   M = [...];          % 零位位姿
   Tsd = [...];        % 期望末端位姿
   thetalist0 = [...]; % 初始关节角度猜测
   eomg = 0.01;        % 角速度容差（rad）
   ev = 0.001;         % 线速度容差（m）
   [thetalist, success] = IKinBodyIterative(Blist, M, Tsd, thetalist0, eomg, ev);
   ```
3. 函数将返回计算得到的关节角度`thetalist`和求解是否成功的标志`success`。

**示例调用**：
```matlab
% 假设Blist和M已经定义
Tsd = ...; % 期望末端位姿
thetalist0 = zeros(6, 1); % 初始猜测
[eomg, ev] = [0.01, 0.001]; % 容差
[thetalist, success] = IKinBodyIterative(Blist, M, Tsd, thetalist0, eomg, ev);
if success
    disp('Inverse kinematics solution found:');
    disp(thetalist);
else
    disp('Failed to find inverse kinematics solution.');
end
```

#### 2. Exercise 2: 平面刚体运动求解器

**功能说明**：计算点P的位置和速度、齐次变换矩阵T_sb、空间旋量Vs和物体旋量Vb，并验证它们之间的关系。

**使用方法**：

1. 打开Matlab，将工作目录设置为`Code`文件夹。
2. 在Matlab命令窗口中输入`Exercise2_Solver`，然后按Enter键。
3. 程序将自动计算并显示所有结果，包括：
   - 点P的位置和速度
   - 齐次变换矩阵T_sb
   - 空间旋量Vs和物体旋量Vb
   - 旋量关系验证

**参数说明**：
程序中已定义以下参数，您可以根据需要修改：
- `L = 1.0;`：旋转中心到坐标系原点的距离（米）
- `d = 0.4;`：点P到旋转中心的距离（米）
- `theta_deg = 30;`：旋转角度（度）
- `theta_dot = 1.0;`：角速度（弧度/秒）

#### 3. Exercise 3: 基于空间雅可比的逆运动学求解器

**功能说明**：实现基于空间雅可比矩阵的逆运动学求解器，用于SCARA机器人的逆运动学求解和轨迹规划。

**使用方法**：

1. 打开Matlab，将工作目录设置为`Code`文件夹。
2. 在Matlab命令窗口中调用`IKinSpaceIterative`函数，输入以下参数：
   ```matlab
   Slist = [...];      % 空间坐标系下的螺旋轴
   M = [...];          % 零位位姿
   Tsd = [...];        % 期望末端位姿
   thetalist0 = [...]; % 初始关节角度猜测
   eomg = 0.01;        % 角速度容差（rad）
   ev = 0.001;         % 线速度容差（m）
   [thetalist, success] = IKinSpaceIterative(Slist, M, Tsd, thetalist0, eomg, ev);
   ```
3. 函数将返回计算得到的关节角度`thetalist`和求解是否成功的标志`success`。

**轨迹规划使用方法**：
```matlab
% 定义初始和目标关节角度
theta_start = [...];
theta_end = [...];
% 生成轨迹
trajectory = generateTrajectory(theta_start, theta_end, t_total, N);
```

**参数说明**：
- `Slist`：空间坐标系下的螺旋轴矩阵，6×n（n为关节数量）
- `M`：零位位姿的齐次变换矩阵
- `Tsd`：期望末端位姿的齐次变换矩阵
- `thetalist0`：初始关节角度猜测
- `eomg`：角速度容差（弧度）
- `ev`：线速度容差（米）
- `t_total`：轨迹总时间（秒）
- `N`：轨迹点数

## 理论部分和代码结合的点

### Exercise 1: 逆运动学求解器

1. **正运动学实现**：
   - 理论：使用指数积（Product of Exponentials, PoE）公式计算末端位姿。
   - 代码：`FKinBody`函数实现了PoE公式，通过循环计算每个关节变换的乘积。
   ```matlab
   function T = FKinBody(M, Blist, thetalist)
       T = M;
       for i = 1:length(thetalist)
           T = T * MatrixExp6(VecTose3(Blist(:, i) * thetalist(i)));
       end
   end
   ```

2. **雅可比矩阵计算**：
   - 理论：根据物体坐标系下的螺旋轴计算雅可比矩阵。
   - 代码：`JacobianBody`函数实现了雅可比矩阵的计算，使用伴随变换矩阵更新螺旋轴。
   ```matlab
   function Jb = JacobianBody(Blist, thetalist)
       Jb = Blist;
       T = eye(4);
       for i = length(thetalist)-1:-1:1
           T = T * MatrixExp6(VecTose3(-Blist(:, i+1) * thetalist(i+1)));
           Jb(:, i) = Adjoint(T) * Blist(:, i);
       end
   end
   ```

3. **误差旋量计算**：
   - 理论：使用矩阵对数将位姿误差转换为旋量表示。
   - 代码：使用`MatrixLog6`和`se3ToVec`函数计算误差旋量。
   ```matlab
   Vb = se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd));
   ```

4. **牛顿-拉夫森迭代**：
   - 理论：通过雅可比矩阵伪逆更新关节角度，最小化位姿误差。
   - 代码：使用`pinv`函数计算伪逆，然后更新关节角度。
   ```matlab
   thetalist = thetalist + pinv(Jb) * Vb;
   ```

### Exercise 2: 平面刚体运动求解器

1. **位置和速度计算**：
   - 理论：根据旋转中心和旋转角度计算点P的位置，对时间求导得到速度。
   - 代码：直接实现了推导得到的解析公式。
   ```matlab
   p_P = [L + d*sin(theta); L - d*cos(theta); 0];
   v_P = [d * theta_dot * cos(theta); d * theta_dot * sin(theta); 0];
   ```

2. **齐次变换矩阵**：
   - 理论：由旋转矩阵和位置向量组成齐次变换矩阵。
   - 代码：构建了旋转矩阵R_sb和位置向量p_P，然后组合成齐次变换矩阵T_sb。
   ```matlab
   R_sb = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
   T_sb = [R_sb, p_P; 0, 0, 0, 1];
   ```

3. **旋量计算**：
   - 理论：根据旋量的定义，计算空间旋量Vs和物体旋量Vb。
   - 代码：分别计算了解析解和使用伴随变换得到的数值解，并进行了验证。
   ```matlab
   Vs_analytical = [0; 0; theta_dot; theta_dot*L; -theta_dot*L; 0];
   Vb_analytical = [0; 0; theta_dot; theta_dot*d; 0; 0];
   ```

4. **伴随变换验证**：
   - 理论：空间旋量和物体旋量通过伴随变换矩阵联系，即\(V_s = [\text{Ad}_{T_{sb}}] V_b\)。
   - 代码：使用`Adjoint`函数计算伴随变换矩阵，验证了解析解和数值解的一致性。
   ```matlab
   Ad_Tsb = Adjoint(T_sb);
   Vs_calculated = Ad_Tsb * Vb_analytical;
   fprintf('Difference between analytical and calculated Vs: %.10f\n', norm(Vs_analytical - Vs_calculated));
   ```

### Exercise 3: 基于空间雅可比的逆运动学求解器

1. **空间雅可比矩阵计算**：
   - 理论：根据空间坐标系下的螺旋轴计算雅可比矩阵，通过伴随变换矩阵更新螺旋轴。
   - 代码：实现了空间雅可比矩阵的计算。
   ```matlab
   function Js = JacobianSpace(Slist, thetalist)
       Js = zeros(6, length(thetalist));
       T = eye(4);
       for i = 1:length(thetalist)
           Js(:, i) = Adjoint(T) * Slist(:, i);
           T = T * MatrixExp6(VecTose3(Slist(:, i) * thetalist(i)));
       end
   end
   ```

2. **空间逆运动学实现**：
   - 理论：使用牛顿-拉夫森迭代法，通过空间雅可比矩阵伪逆更新关节角度。
   - 代码：实现了基于空间雅可比的逆运动学求解器。
   ```matlab
   function [thetalist, success] = IKinSpaceIterative(Slist, M, Tsd, thetalist0, eomg, ev)
       % 实现基于空间雅可比的逆运动学求解
       % ...
       Ts = FKinSpace(M, Slist, thetalist);
       Vb = se3ToVec(MatrixLog6(Ts * TransInv(Tsd)));
       Js = JacobianSpace(Slist, thetalist);
       thetalist = thetalist + pinv(Js) * Vb;
       % ...
   end
   ```

3. **轨迹规划**：
   - 理论：使用三次时间缩放函数生成平滑的关节空间轨迹。
   - 代码：实现了轨迹生成函数。
   ```matlab
   function trajectory = generateTrajectory(theta_start, theta_end, t_total, N)
       % 生成平滑的关节空间轨迹
       % ...
       t = linspace(0, t_total, N);
       s = cubicTimeScaling(t_total, t);
       trajectory = theta_start + s * (theta_end - theta_start);
       % ...
   end
   ```

4. **三次时间缩放**：
   - 理论：三次多项式时间缩放函数，生成平滑的加速和减速过程。
   - 代码：实现了三次时间缩放函数。
   ```matlab
   function s = cubicTimeScaling(t_total, t)
       s = 3*(t/t_total)^2 - 2*(t/t_total)^3;
   end
   ```

## 代码特点

1. **模块化设计**：代码结构清晰，功能模块化，便于理解和维护。
2. **相对路径调用**：使用相对路径调用编程库中的函数，无需将整个库复制到工作目录。
3. **详细注释**：代码中包含详细的注释，解释了每个函数的功能、参数和返回值。
4. **错误处理**：逆运动学求解器包含了收敛判断和最大迭代次数限制，避免无限循环。
5. **验证机制**：Exercise 2的求解器包含了解析解和数值解的验证，确保结果的正确性。

## 注意事项

1. 运行代码前，请确保Matlab可以访问`编程库/Industrial_Robotics_Library`文件夹中的函数。
2. 对于逆运动学求解器，初始猜测的选择会影响求解结果和收敛速度，建议选择合理的初始猜测。
3. 旋量计算中，需要注意坐标系的定义和变换关系，确保旋转中心和点P的位置关系正确。
4. 奇异性分析时，需要考虑机器人的关节配置和雅可比矩阵的秩，找到雅可比矩阵秩不足的情况。

## 参考文献

[1] Lynch, K. M., & Park, F. C. (2017). Modern Robotics: Mechanics, Planning, and Control. Cambridge University Press.
