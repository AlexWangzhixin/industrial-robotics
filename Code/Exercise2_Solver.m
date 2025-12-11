% Exercise 2: Planar Rigid Body Motion - Validation Script
% Corresponding to LaTeX Report Exercise 2
% Date: December 8, 2025

clear; clc;
addpath('../编程库/Industrial_Robotics_Library');

%% Parameters
L = 1.0;            % meters
d = 0.4;            % meters
theta_deg = 30;     % degrees
theta = deg2rad(theta_deg);
theta_dot = 1.0;    % rad/s

fprintf('--- Exercise 2 Verification ---\n');

%% 2.1 & 2.2: T_sb Calculation
% Position of P (Origin of {b})
p_P = [L + d*sin(theta); 
       L - d*cos(theta); 
       0];

% Rotation Matrix R_sb
R_sb = [cos(theta), -sin(theta), 0;
        sin(theta),  cos(theta), 0;
        0,           0,          1];

% Homogeneous Transformation Matrix
T_sb = [R_sb, p_P; 0 0 0 1];

fprintf('T_sb (Transformation Matrix):\n');
disp(T_sb);

%% 2.3 Twist Calculation

% --- Analytical Spatial Twist Vs ---
% From derivation: Vs = [0; 0; theta_dot; theta_dot*L; -theta_dot*L; 0]
omega_s = [0; 0; theta_dot];
v_s = [theta_dot * L; -theta_dot * L; 0];
Vs_analytical = [omega_s; v_s];

% --- Analytical Body Twist Vb ---
% From derivation: Vb = [0; 0; theta_dot; theta_dot*d; 0; 0]
omega_b = [0; 0; theta_dot];
v_b = [theta_dot * d; 0; 0];
Vb_analytical = [omega_b; v_b];

fprintf('Vs (Analytical): \n'); disp(Vs_analytical');
fprintf('Vb (Analytical): \n'); disp(Vb_analytical');

%% 2.4 Verification: Vs = Ad(T_sb) * Vb
Vs_from_Vb = Adjoint(T_sb) * Vb_analytical;

fprintf('Vs calculated from Ad(T_sb)*Vb:\n'); 
disp(Vs_from_Vb');

error_norm = norm(Vs_analytical - Vs_from_Vb);
if error_norm < 1e-10
    fprintf('[SUCCESS] Verification passed! Analytical derivations match geometric transforms.\n');
else
    fprintf('[FAILURE] Verification failed. Error: %e\n', error_norm);
end

% Verify Vb = Ad(T_bs) * Vs as well
T_bs = TransInv(T_sb);
Vb_from_Vs = Adjoint(T_bs) * Vs_analytical;
error_norm_b = norm(Vb_analytical - Vb_from_Vs);

fprintf('Diff in Vb verification: %e\n', error_norm_b);