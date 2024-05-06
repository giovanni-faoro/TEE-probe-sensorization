function Rvi = ChardonnensLinearSolution(omega_imu,omega_vic,dt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Linear solution to the Chardonnens alignment problem of the equation (6)
% This solution can be used to provide an inittial condition for the
% iterative optimization algorithm
%
%   INPUT:      omega_vic = angular velocity measured with VICON
%               omega_imu = angular velocity measured with IMU
%               dt = sampling time (it is assumed equal for VICON and IMU)
%
%   OUTPUT:     Rvi = Rotation matrix from IMU local frame to the VICON
%                     local frame
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% See Chardonnens 2012
IMU_theta = dt*cumsum(omega_imu,1); % Chardonnens eq 4
VIC_theta = dt*cumsum(omega_vic,1); % Chardonnens eq 5

%% Initial condition -  linear least squares solution
N = length(omega_vic);
y = reshape(IMU_theta',3*N,1);

X = zeros(3*N,9);

X(1:3:end,1:3) = VIC_theta;
X(2:3:end,4:6) = VIC_theta;
X(3:3:end,7:9) = VIC_theta;

% Least square solution to y = X*beta
beta = X\y;

Riv0 = reshape(beta,3,3);
Riv0 = Riv0/norm(Riv0);

% Orthonormality constraint (brute-force approach) Heikkila 2000 camera calibration
[U , ~, V] = svd(Riv0);
Rvi = (U*eye(3)*V')';       %% Note the transposition!

return