clear all; close all; clc;
addpath('../Industrial_Robotics_Library/');%清空命令行窗口并添加路径

%对第一问的分析
L1 = 0.3;%Upper leg，单位是米
L2 = 0.4;%Lower leg
L0 = 0.4;%CoM到Joint 1的距离
%建立{S}、{F}、端点三个坐标系之间的平移关系
p_J1_S = [L0; 0; 0];% 关节1在空间坐标系{S}中的位置
p_J1_F = [0; 0.5; 0];% 关节1在足端坐标系{F}中的位置
p_F_J1 = -p_J1_F;%足端原点在关节1坐标系中的位置


S1 = [0; 0; 1; cross(-[0;0;1], [L0; 0; 0])];       % Joint 1 Screw
S2 = [0; 0; 1; cross(-[0;0;1], [L0+L1; 0; 0])];    % Joint 2 Screw
Slist = [S1, S2];

%椭圆轨迹的相关参数
x1 = 0; y1 = 0;%{F}坐标系中的轨迹起点
x2 = 0.25; y2 = 0.2;%{F}坐标系中的轨迹终点
e = 0.9;%该椭圆的偏心率
%对相关的参数进行计算，如半长轴等
dist = sqrt((x2-x1)^2 + (y2-y1)^2);%起点到终点的直线距离
a = dist / 2;%椭圆半长轴
b = a * sqrt(1 - e^2);%椭圆半短轴
w = atan2(y2-y1, x2-x1);%相对于X轴的旋转角
%输出计算后的结果
fprintf('a = %.4f, b = %.4f, w = %.4f rad\n', a, b, w);

%对椭圆的轨迹参数方程进行定义 
% (beta from pi to 0)
% Returns position relative to {F} origin
get_traj = @(beta) [(x1+x2)/2 + a*cos(beta)*cos(w) - b*sin(beta)*sin(w);(y1+y2)/2 + a*cos(beta)*sin(w) + b*sin(beta)*cos(w);0]; 
%椭圆在平面运动，所以x坐标(x1+x2)/2 + a*cos(beta)*cos(w) - b*sin(beta)*sin(w)
%Y坐标(y1+y2)/2 + a*cos(beta)*sin(w) + b*sin(beta)*cos(w)
%z坐标=0

% 绘制foot的轨迹
beta_plot = linspace(0, 2*pi, 200);%参数点200，数量可调
traj_points = zeros(3, length(beta_plot));
for i = 1:length(beta_plot)
    traj_points(:,i) = get_traj(beta_plot(i));
end

figure(1); %对图片进行设置
plot(traj_points(1,:), traj_points(2,:), 'b--'); hold on;
plot(x1, y1, 'ko', 'MarkerFaceColor', 'k'); text(x1, y1-0.05, 'Start');
plot(x2, y2, 'ro', 'MarkerFaceColor', 'r'); text(x2, y2+0.05, 'End');
axis equal; grid on;
title('Exercise 3.1: Foot Trajectory (in Foot Frame)');
xlabel('x [m]'); ylabel('y [m]');%设置单位

%2. Analytical Inverse Kinematics (3.2)
fprintf('\n3.2 Analytical IK\n');

% 起点和终点在{F}坐标系中的表示
P1_F = [x1; y1; 0];% Start position in {F}
P2_F = [x2; y2; 0];% End position in {F}
%转换到关节1的坐标系{J1}
P1_J1 = P1_F + p_F_J1;% in {J1}
P2_J1 = P2_F + p_F_J1;% in {J1}

M = [1, 0, 0, L0 + L1 + L2;
     0, 1, 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 1];

% 数值解析方法求解逆运动学，暂未在library找到相关公式，定义一个新公式：
%function theta = analytical_IK(x, y, L1, L2)
theta_start = IK_2R_IKinBody(P1_J1(1), P1_J1(2), L1, L2);
theta_end = IK_2R_IKinBody(P2_J1(1), P2_J1(2), L1, L2);

%输出结果
fprintf('Configuration 1 (Start): x=%.2f, y=%.2f theta1=%.2f deg, theta2=%.2f deg\n', ...
    P1_J1(1), P1_J1(2), rad2deg(theta_start(1)), rad2deg(theta_start(2)));
fprintf('Configuration 2 (End):   x=%.2f, y=%.2f theta1=%.2f deg, theta2=%.2f deg\n', ...
    P2_J1(1), P2_J1(2), rad2deg(theta_end(1)), rad2deg(theta_end(2)));

%3. Velocity Search and Simulation(3.3, 3.4, 3.5)
fprintf('\n3.3 & 3.4 Numerical IK and Velocity Check\n');

theta_dot_max_limit = 5.0; % 关节最大允许的角速度约束[rad/s]
beta_dot_3 = 1.0;      %使用第3问中的数据 [rad/s]
theta_guess = [deg2rad(-30); deg2rad(60)];

%历史数据存储结构体
history.time = [];%用于存储方针时间序列
history.pos_S = [];% Foot position in Space Frame空间框架中的足部位置
history.theta = [];% Joint angles关节角度
history.vel_foot = [];% Foot Cartesian Velocity脚笛卡尔速度
history.vel_joint = [];% Joint Velocities关节速度

found_valid_beta_dot = false;%是否找到满足约束的beta_dot
beta_dot = beta_dot_3;%初始化beta_dot为beta_dot_3

while ~found_valid_beta_dot%若当前beta_dot不满足约束，就持续搜索
    dt = 0.02; %仿真时间步长
    T_total = pi / beta_dot; %轨迹总时间，由 beta 从 π 变化到 0 决定
    time = 0:dt:T_total;%时间序列
    
    max_q_vel = 0;%记录最大关节速度
    valid_run = true;
    theta_curr = theta_guess; % Reset theta for new run

    for i = 1:length(time)
        t = time(i);% 当前仿真时刻
        beta = pi - beta_dot * t; % 当前椭圆轨迹参数 beta（线性随时间变化）
        
        %期望位置和速度
        p_d_F = get_traj(beta);% 当前时刻足端在 {F} 坐标系中的期望位置
        p_d_J1 = p_d_F + p_F_J1;%转换到关节1坐标系
        
        %由椭圆导数求笛卡尔速度（链式法则）
        dx_dbeta = -a*sin(beta)*cos(w) - b*cos(beta)*sin(w);%x速度
        dy_dbeta = -a*sin(beta)*sin(w) + b*cos(beta)*cos(w);%y速度
        v_d_J1 = [dx_dbeta; dy_dbeta] * (-beta_dot);%求导（链式法则）
        
        % T_sd = [eye(3), p_S; 0 0 0 1];
        
        % 当前足端位置对应的关节角，暂未在library找到相关公式，
        % 使用定义的新公式：analytical_IK
        theta_curr = IK_2R_IKinBody(p_d_J1(1), p_d_J1(2), L1, L2);
        
        %计算二维 2R 机械臂的雅可比矩阵，为方便计算，此处定义一个新公式
        Js = JacobianSpace(Slist, theta_curr);
        J_linear = Js(4:6, :);

        % 通过解线性方程组求解关节角速度
        % q_dot = Js \ v_d_J1;
        
        % 检查奇点
        if rcond(J_linear'*J_linear) < 1e-4
            valid_run = false; break; % Singularity
        else
            q_dot = J_linear \ v_cart_F;
        end
        
        % Check max velocity constraint
        current_max_vel = max(abs(q_dot));
        if current_max_vel > max_q_vel
            max_q_vel = current_max_vel;
        end
        
        if current_max_vel > theta_dot_max_limit
            %若超过关节速度限制，则判定失败
            valid_run = false;
            break;
        end
    end
    
    fprintf('Testing beta_dot = %.2f rad/s Max Joint Vel = %.2f rad/s\n', ...
        beta_dot, max_q_vel);
    % 输出当前 beta_dot 的测试结果
    if valid_run
        found_valid_beta_dot = true;
        fprintf('SUCCESS beta_dot = %.2f satisfies constraints.\n', beta_dot);
    else
        % 若失败，则减小 beta_dot 重新测试
        beta_dot = beta_dot * 0.8;
    end
end

%4.完整轨迹生成与可视化
dt = 0.02;%仿真时间步长 
T_total = pi / beta_dot;%总轨迹时间
time = 0:dt:T_total;%时间序列

% Define Screw Axes for Home Config (Horizontal Extension)
% J1 at (L0,0), J2 at (L0+L1,0). Rot axes = Z (0,0,1).
S1 = [0; 0; 1; cross(-[0;0;1], [L0; 0; 0])];       % [0 0 1 0 -L0 0]
%第一关节的空间螺旋轴
S2 = [0; 0; 1; cross(-[0;0;1], [L0+L1; 0; 0])];    % [0 0 1 0 -(L0+L1) 0]
% 第二关节的空间螺旋轴
Slist = [S1, S2];% 构造螺旋轴矩阵，用于空间雅可比计算

for i = 1:length(time)
    t = time(i);
    beta = pi - beta_dot * t;%当前轨迹参数
    
    %位置
    p_d_F = get_traj(beta);% 足端在 {F} 坐标系中的位置
    p_d_J1 = p_d_F + p_F_J1;% 转换到 {J1}
    p_d_S = p_d_J1 + p_J1_S;% 转换到空间坐标系 {S}
    
    % Inverse Kinematics，使用函数function theta = analytical_IK(x, y, L1, L2)
    theta_sol = analytical_IK(p_d_J1(1), p_d_J1(2), L1, L2);
    
    %速度计算
    dx_dbeta = -a*sin(beta)*cos(w) - b*cos(beta)*sin(w);
    dy_dbeta = -a*sin(beta)*sin(w) + b*cos(beta)*cos(w);
    v_cart_F = [dx_dbeta; dy_dbeta; 0] * (-beta_dot);% 足端在 {F} 中的线速度
    
    % Joint Velocity using Library JacobianSpace
    Js = JacobianSpace(Slist, theta_sol);%调用函数
    
    % Extract linear part of Jacobian and solve for joint velocities
    J_linear = Js(4:6, :);% 提取线速度部分的雅可比
    if det(J_linear'*J_linear) < 1e-6
        warning('Near Singularity at t=%.2f', t);
        theta_dot = [0;0];
    else
        theta_dot = pinv(J_linear) * v_d_J1;% 使用伪逆求解关节速度
    end
    
    % Store Data
    history.time = [history.time, t];% 记录时间
    history.pos_S = [history.pos_S, p_d_S];% 记录空间位置
    history.theta = [history.theta, theta_sol];% 记录关节角
    history.vel_foot = [history.vel_foot, v_cart_F];% 记录足端速度
    history.vel_joint = [history.vel_joint, theta_dot];% 记录关节速度
end

fprintf('\nSimulation 1Complete.\n');

%5. Results Visualization
% Plot Foot Position and Joint Angles vs Time
figure(2);
subplot(2,1,1);
% 创建 2 行 1 列子图布局，激活第 1 个子图
plot(history.time, history.pos_S(1,:), 'r', 'LineWidth', 1.5); hold on;
% 绘制足端在空间坐标系 {S} 中 x 方向的位置随时间变化（红色曲线）% 保持当前子图，允许叠加绘制
plot(history.time, history.pos_S(2,:), 'b', 'LineWidth', 1.5);
% 绘制足端在空间坐标系 {S} 中 y 方向的位置随时间变化（蓝色曲线）
title('Foot Position w.r.t CoM Frame');
% 设置子图标题：足端相对于质心坐标系的位置变化
legend('x_s', 'y_s'); grid on; xlabel('Time [s]'); ylabel('Position [m]');
% 设置图例，表示空间坐标系下的 x、y 分量

subplot(2,1,2);% 激活第 2 个子图（关节角变化）
plot(history.time, rad2deg(history.theta(1,:)), 'r', 'LineWidth', 1.5); hold on;
% 绘制第一个关节角 theta1 随时间变化（弧度 → 角度）
plot(history.time, rad2deg(history.theta(2,:)), 'b', 'LineWidth', 1.5);
% 绘制第二个关节角 theta2 随时间变化
title('Joint Angles vs Time');
% 设置子图标题：关节角随时间变化
legend('\theta_1', '\theta_2'); grid on; xlabel('Time [s]'); ylabel('Angle [deg]');
% 设置图例，标识两个关节角

%3D Visualization (3.5)
figure('Name', 'Exercise 3.5: 3D Leg Trajectory', 'Color', 'w');
% 设置图例，标识两个关节角
hold on; grid on; axis equal;
% 保持绘图、开启网格、设置坐标轴比例一致（防止三维形变）
view(3);
% 设置三维视角
xlabel('X_S [m]'); ylabel('Y_S [m]'); zlabel('Z_S [m]');
title('3D Trajectory and Velocity Vectors');
% 设置三维图标题：轨迹与速度矢量

% Draw Robot Body (Schematic)
plot3([-0.2, 0.6], [0, 0], [0, 0], 'k-', 'LineWidth', 5);
% 使用粗黑线绘制机器人机身示意（沿 X 方向）
text(0, 0, 0.1, 'Body (CoM)');
% 在机身上方标注“Body (CoM)”表示质心位置

% Draw Trajectory
plot3(history.pos_S(1,:), history.pos_S(2,:), history.pos_S(3,:), 'b--', 'LineWidth', 1.5);
% 绘制足端在空间坐标系 {S} 中的三维运动轨迹（蓝色虚线）

% Plot Velocity Vectors at selected points
n_steps = length(history.time);% 仿真总步数
skip = 5; % 每隔 5 个时间步绘制一次速度矢量，减少视觉拥挤

for i = 1:skip:n_steps % 按时间序列循环绘制动画
    q = history.theta(:, i); % 当前时刻的关节角向量 [theta1; theta2]
    pos = history.pos_S(:, i); % 当前时刻足端在空间坐标系中的位置
    vel = history.vel_foot(:, i);% 当前时刻足端的线速度向量
    
    % Forward Kinematics to get joint positions for plotting
    p0 = p_J1_S;% 关节 1 在空间坐标系中的位置
    p1 = p0 + [L1*cos(q(1)); L1*sin(q(1)); 0];%第二关节位置
    p2 = pos; % End effector position
    
    % Plot Links
    h_links = plot3([p0(1) p1(1) p2(1)], [p0(2) p1(2) p2(2)], [0 0 0], 'o-', ...
        'Color', [0.8 0.2 0.2], 'LineWidth', 2, 'MarkerFaceColor', 'k');
    % 绘制两段连杆及关节点

    % Plot Velocity Vector
    h_vel = quiver3(p2(1), p2(2), p2(3), vel(1), vel(2), vel(3), ...
        0.5, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
     %绘制足端速度方向与大小
    drawnow; % 强制 MATLAB 立即刷新图像，实现动画效果
    
    % Clear specific handles for animation effect
    if i < n_steps - skip
        delete(h_links);
        delete(h_vel);
    end
end

%% Helper Functions

function theta = analytical_IK(x, y, L1, L2)
    % Solves 2R IK for x,y in the base frame of the chain
    % Returns [theta1; theta2]
    
    % Check workspace
    r = sqrt(x^2 + y^2);
    if r > (L1 + L2) || r < abs(L1 - L2)
        warning('Target out of workspace');
        theta = [NaN; NaN];
        return;
    end
    
    % Law of cosines for theta2
    cos_t2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);
    
    % Safety for acos
    cos_t2 = max(min(cos_t2, 1.0), -1.0);
    
    % Choose knee-backward solution (theta2 < 0 for dog leg)
    theta2 = -acos(cos_t2);
    
    % Calculate theta1
    k1 = L1 + L2*cos(theta2);
    k2 = L2*sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);
    
    theta = [theta1; theta2];
end

function J = get_jacobian_2R(q, L1, L2)
    % Calculate 2R planar robot Jacobian matrix
    t1 = q(1);
    t2 = q(2);
    s1 = sin(t1); c1 = cos(t1);
    s12 = sin(t1+t2); c12 = cos(t1+t2);
    
    J = [ -L1*s1 - L2*s12,  -L2*s12;
           L1*c1 + L2*c12,   L2*c12 ];
end

function theta = IK_2R_IKinBody(x, y, L1, L2)
% 用 IKinBody 实现 2R 平面机械臂的逆运动学
% 输入:
%   x, y : 足端在 {J1} 坐标系下的位置
% 输出:
%   theta = [theta1; theta2]

    % ===== 1. 构造目标位姿 T_sd =====
    % 平面问题，不关心姿态，固定为单位旋转
    R = eye(3);
    p = [x; y; 0];
    T_sd = [R, p;
            0 0 0 1];

    % ===== 2. Home configuration M =====
    % theta1 = theta2 = 0 时，末端在 (L1+L2, 0)
    M = [eye(3), [L1+L2; 0; 0];
         0 0 0 1];

    % ===== 3. Body screw axes =====
    % 关节轴均为 z 轴
    % Body frame 在末端
    B1 = [0; 0; 1; 0;  L1+L2; 0];
    B2 = [0; 0; 1; 0;      L2; 0];
    Blist = [B1, B2];

    % ===== 4. 初始猜测（控制膝盖方向）=====
    % 对应你 analytical_IK 中的 “knee-backward”
    thetalist0 = [0.1; -0.5];

    % ===== 5. 误差容限 =====
    eomg = 1e-4;
    ev   = 1e-4;

    % ===== 6. 调用 IKinBody =====
    [theta, success] = IKinBody(Blist, M, T_sd, thetalist0, eomg, ev);

    if ~success
        warning('IKinBody failed to converge');
        theta = [NaN; NaN];
    end
end
