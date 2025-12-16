%% Exercise 1 Verification Script
clear; close all; clc;

addpath('../Industrial_Robotics_Library/');
%% 1. 定义机器人参数 (请根据 Fig 4 修改此处!)
% 示例: 这是一个平面 3R 机器人的参数，你需要根据作业图片修改为实际的空间机器人参数。
L1 = 1; L2 = 1; L3 = 1;

% 定义螺旋轴 (Slist) - 每一列是一个 [omega; v]
% 示例: 3个旋转关节，都在Z轴旋转
S1 = [0; 0; 1; 0; 0; 0];           % 关节1位于原点
S2 = [0; 0; 1; 0; -L1; 0];         % 关节2位于 (L1, 0, 0) -> v = -w x q
S3 = [0; 0; 1; 0; -(L1+L2); 0];    % 关节3位于 (L1+L2, 0, 0)
Slist = [S1, S2, S3];

% 定义 Home Configuration (M) - 关节角为0时的末端位姿
M = [1, 0, 0, L1+L2+L3;
     0, 1, 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 1];

%% 2. 生成测试目标 (Forward Kinematics -> Inverse Kinematics)
% 为了验证求解器，我们先设定一个已知的关节角，算出T_sd，再看能否反解回去。

fprintf('开始验证逆运动学求解器...\n');
num_tests = 5; % 测试次数
eomg = 1e-3;   % 角度容差
ev = 1e-3;     % 位置容差

for k = 1:num_tests
    fprintf('\n--- 测试案例 %d ---\n', k);
    
    % A. 随机生成目标关节角 (Ground Truth)
    theta_target = -pi + 2*pi*rand(3, 1); % [-pi, pi] 之间
    
    % B. 计算对应的末端位姿 (T_sd)
    T_sd = FKinSpace(M, Slist, theta_target);
    
    % C. 设置初始猜测 (Initial Guess)
    %    稍微偏离目标值，模拟真实情况
    theta0 = theta_target + 0.5 * (rand(3,1) - 0.5); 
    
    % D. 调用你的求解器
    [theta_sol, success, err_hist] = IKinBodyIterative(Slist, M, T_sd, theta0, eomg, ev);
    
    % E. 验证结果
    if success
        % 计算解算出的关节角对应的位姿
        T_sol = FKinSpace(M, Slist, theta_sol);
        
        % 计算位姿误差矩阵
        T_error = inv(T_sol) * T_sd;
        position_error = norm(T_error(1:3, 4));
        fprintf('结果: 收敛成功!\n');
        fprintf('目标关节角: [%.3f, %.3f, %.3f]\n', theta_target);
        fprintf('求解关节角: [%.3f, %.3f, %.3f]\n', theta_sol);
        fprintf('末端位置误差: %.6f m\n', position_error);
        
        % 绘制误差收敛曲线 (可选)
        % figure(k); plot(err_hist, '-o'); title(['Convergence Test ' num2str(k)]); grid on;
        % xlabel('Iteration'); ylabel('Error Norm');
    else
        fprintf('结果: 收敛失败 (可能是奇异点或超出工作空间)\n');
    end
end

fprintf('\n所有验证完成。\n');