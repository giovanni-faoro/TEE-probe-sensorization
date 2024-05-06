function angles = fromMatrixToAngle(R, sequence)
% This function returns cardan angle from a 3x3xF or 4x4xF matrix, where F 
% is the number of frames. 

    % Check for inputs
    if nargin ~= 2 
        error('Wrong number of input parameters')
    end
        
    % Prepare output
    psi = []; %#ok<NASGU>
    theta = [];
    phi = [];
    switch sequence
        case 'x'
            psi = asin(R(3,2,:));
        case 'y'
            psi = asin(R(1,3,:));
        case 'z'
            psi = asin(R(2,1,:));
        case 'xy'
            psi = asin(R(3,2,:));
            theta = asin(R(1,3,:));
        case 'xz'
            psi = -asin(R(2,3,:));
            theta = -asin(R(1,2,:));
        case 'yx'
            psi = -asin(R(3,1,:));
            theta = -asin(R(2,3,:));
        case 'yz'
            psi = asin(R(1,3,:));
            theta = asin(R(2,1,:));
        case 'zx'
            psi = asin(R(2,1,:));
            theta = asin(R(3,2,:));
        case 'zy'
            psi = -asin(R(1,2,:));
            theta = -asin(R(3,1,:));
        case 'xyz'
            psi   = atan2(-R(2,3,:), R(3,3,:)); 	
            theta = asin(R(1,3,:)); 				
            phi   = atan2(-R(1,2,:), R(1,1,:));
        case 'xzy'   
            psi   = atan2(R(3,2,:),R(2,2,:));       
            phi = atan2(R(1,3,:), R(1,1,:));   
            theta   = asin(-R(1,2,:));
        case 'yxz'
            theta   = asin(-R(2,3,:));
            psi = atan2(R(1,3,:), R(3,3,:));
            phi   = atan2(R(2,1,:), R(2,2,:));
        case 'yzx'
            phi   = atan2(-R(2,3,:), R(2,2,:));
            psi = atan2(-R(3,1,:), R(1,1,:));
            theta   = asin(R(2,1,:));        
        case 'zxy'
            theta   = asin(R(3,2,:));
            phi = atan2(-R(3,1,:), R(3,3,:));
            psi   = atan2(-R(1,2,:), R(2,2,:));
        case 'zyx'
            phi   = atan2(R(3,2,:), R(3,3,:));
            theta = asin(-R(3,1,:));
            psi   = atan2(R(2,1,:), R(1,1,:));
        case 'zyz'
            psi   = atan2(R(2,3,:), R(1,3,:));
            theta = acos(R(3,3,:));
            phi   = atan2(R(3,2,:), -R(3,1,:));
        case 'zxz'
            psi	  = atan2(R(1,3,:), -R(2,3,:));
            theta = acos(R(3,3,:));
            phi   = atan2(R(3,1,:), R(3,2,:));
        otherwise
            error('Séquence d''angle incorrecte dans angleRotation')
    end
    
    % Prepare output
    angles = nan(length(sequence), size(R, 3));
    if ~isempty(psi)
        angles(1, :) = real(psi);
    end
    if ~isempty(theta)
        angles(2, :) = real(theta);
    end
    if ~isempty(phi)
        angles(3, :) = real(phi);
    end
    
end
