function [omega,quat_derivative,quat_derivative2] = quaternion2AngularVelocityI(quaternion,dt)
    %This function returns the estimated angular velocity in the inertial reference frame from a sequence
    %of quaternion vectors
    %   INPUT:  quaternion, should be a 4xn matrix of n quaternions in the
    %   form (w,x,y,z)
    %           dt, sampling time
    %   OUTPUT: omega, 3x(n-1) angular velocity matrix
    
    %The angular velocity vector is retrieved from quaternions based on the
    %formula:           omega(t) = lim_dt->0 2Im((q(t+dt)q*(t))/dt)
    %This formula can be checked knowing that the quaternion derivative
    %q(t)_dot = lim_dt->0 ((q(t+dt)-q(t))/dt) should be equal to
    %q(t)_dot = 0.5q(t)*[0;omega(t)]
    
    [M,N] = size(quaternion);
    if (M ~= 4)
        error('Incorrect parameter vector');
    end
    omega = zeros(3,N-1);
    quat_derivative = zeros(4,N-1);
    for i=1:N-1
        %taking the imaginary part of quaternion is equivalent to just skip
        %the first component
        var = (2/dt)*quatmultiply(quaternion(:,i+1)',quatconj(quaternion(:,i)'));
        omega(:,i) = var(2:end);
        quat_derivative(:,i) = (quaternion(:,i+1)-quaternion(:,i))/dt;
    end
    omega_quat = zeros(4,N-1);
    omega_quat(2:4,:) = omega;
    quat_derivative2 = (0.5*quatmultiply(quaternion(:,2:end)',omega_quat'))';
return
