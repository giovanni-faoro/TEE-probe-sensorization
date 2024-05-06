function out = cost_fun_Ch(par,VIC_theta,IMU_theta,dt)
%% ATTENZIONE: nella formula 11 di chardonnens 2012 c'è un errore:
% la matrice di rotazione MCS_R_IMU va trasposta!!!! perchè sto
% moltiplicando a destra, cioè al contrario!!!


%% Check input parameter
if (size(par,1) ~= 1 && size(par,2)~= 9)
    error('Incorrect parameter vector ');
end


% Omega velocity must be Nx3
if size(VIC_theta,1) < size(VIC_theta,2)
    VIC_theta = VIC_theta';
end

if size(IMU_theta,1) < size(IMU_theta,2)
    IMU_theta = IMU_theta';
end

%% Parameter reorganization
dS = par(1:3);
b  = par(4:6);
Rvi = reshape(par(7:15),3,3);

N = length(VIC_theta);

B = dt*cumsum(repmat(b,N,1));

%% Cost function
out = VIC_theta - (IMU_theta*(eye(3) - diag(dS)) - B) * Rvi';

out = norm(norm(out));
