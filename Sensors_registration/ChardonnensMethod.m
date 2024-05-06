function [VICON_R_IMU dS b] = ChardonnensMethod(omega_vic,omega_imu,dt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This function performs Chardonnens' calibration method. A nonlinear
%   optimization algorithm is used to estimate alignment matrix, gyroscopes
%   sensitivity and bias.
%
%   INPUT:      omega_vic = angular velocity measured with VICON
%               omega_imu = angular velocity measured with IMU
%               dt = sampling time (it is assumed equal for VICON and IMU)
%
%   OUTPUT:     VICON_R_IMU = estimated alignment matrix from IMU to VICON 
%                             local frame (3x3)
%               dS    = estimated gyroscope error sensitivity. Gyroscope
%                       sensitivity is 1-dS (on each respective axis) (1x3)
%               b     = gyroscopes bias 1x3
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Input check
if (max(size(omega_vic)) <= 3) ||(max(size(omega_vic)) <= 3)
    error('Time Series too short!');
end

if(length(omega_vic)~= length(omega_imu))
    error('Vectors dimensions mismatch');
    
end

% Omega velocity must be Nx3
if size(omega_vic,1) < size(omega_vic,2)
    omega_vic = omega_vic';
end

if size(omega_imu,1) < size(omega_imu,2)
    omega_imu = omega_imu';
end

%% See Chardonnens 2012
IMU_theta = dt*cumsum(omega_imu,1); % Chardonnens eq 4
VIC_theta = dt*cumsum(omega_vic,1); % Chardonnens eq 5


%% Initial condition -  linear least squares solution
N = length(omega_vic);

Rvi0 = ChardonnensLinearSolution(omega_vic,omega_imu,dt);
sens0 = [0 0 0];
b0 = [0 0 0];

x0 = [sens0 b0 reshape(Rvi0,1,9)];

%% Nonlinear constrained minimization algorithms

% Objective function
obj_fun = @(par)cost_fun_Ch(par,VIC_theta,IMU_theta,dt);

% Orthonormality constraint  
con_fun = @(r) cons_Ch(r);

% Optimization algorithm options
options = optimset('Algorithm','active-set','MaxFunEvals',5000000,'MaxIter',5000000,'TolFun',1e-17,'TolCon',1e-17);
% options = optimset('Algorithm','active-set','MaxFunEvals',5000,'TolFun',1e-15,'TolCon',1e-15);

x = fmincon(obj_fun,x0,[],[],[],[],[],[],con_fun,options);

%% Output
dS = x(1:3);
b = x(4:6);
VICON_R_IMU = reshape(x(7:15),3,3);

return
