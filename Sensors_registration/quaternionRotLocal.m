function [vector_rot] = quaternionRotLocal(quaternion,vector)
    %This function returns the rotated vector after applying quaternion
    %The rotation is applied with respect to local axis
    %   INPUT:  quaternion, should be a 4x1 vector of 1 quaternions in the
    %   form (w,x,y,z)
    %           vector, should be a 3xn matrix of n 3D vector to be rotated
    %   OUTPUT: vector_rot, 3xn rotated vectors
    
    [M,N] = size(vector);
    if (M ~= 3)
        error('Incorrect parameter vector');
    end
    vector_quat = zeros(4,N);
    vector_quat(2:end,:) = vector;
    vector_rot = quatmultiply(quatinv(quaternion'),quatmultiply(vector_quat',quaternion'))';
    vector_rot = vector_rot(2:end,:);
return
