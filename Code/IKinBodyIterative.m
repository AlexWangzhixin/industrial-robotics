function [thetalist, success] = IKinBodyIterative(Blist, M, Tsd, thetalist0, eomg, ev)
% ENumerical Inverse Kinematics Implementation
%
% Inputs:
%   Blist: Body screw axes list (6xn matrix)
%   M: Home configuration of end-effector (4x4 matrix)
%   Tsd: Desired end-effector configuration (4x4 matrix)
%   thetalist0: Initial guess for joint angles
%   eomg: Orientation error tolerance (epsilon)
%   ev: Position error tolerance (epsilon)
%
% Outputs:
%   thetalist: Joint angles solution
%   success: Logical true if converged, false otherwise
    
    addpath('../Industrial_Robotics_Library/');

    thetalist = thetalist0;
    i = 0;
    maxiterations = 20; % Set a reasonable limit
    
    % Step 2: Compute FK for initial guess
    
    Tsb = FKinBody(M, Blist, thetalist);
    
    % Step 3: Compute Error Twist Vb in Body Frame
    % Vb = log(Tsb^-1 * Tsd)
    Vb = se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd));
    
    % Check convergence (Step 6 pre-check)
    err = norm(Vb(1:3)) > eomg || norm(Vb(4:6)) > ev;
    
    while err && i < maxiterations
        i = i + 1;
        
        % Step 4: Compute Body Jacobian
        Jb = JacobianBody(Blist, thetalist);
        
        % Step 5: Update Step using Pseudoinverse
        % theta = theta + pinv(Jb) * Vb
        thetalist = thetalist + pinv(Jb) * Vb;
        
        % Recalculate FK and Error for next iteration
        % M: Home configuration of end-effector
        % Tsd: Desired end-effector configuration
        Tsb = FKinBody(M, Blist, thetalist);
        Vb = se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd));
        err = norm(Vb(1:3)) > eomg || norm(Vb(4:6)) > ev;
    end
    
    success = ~err;
    
    if success
        fprintf('IK Converged in %d iterations.\n', i);
    else
        fprintf('IK Failed to converge after %d iterations.\n', maxiterations);
    end
end