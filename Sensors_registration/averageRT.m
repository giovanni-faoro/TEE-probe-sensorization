function RTmean = averageRT(RT, ignoreNaN)
    % Chose an arbitrary angle sequence to convert into angle
    seq = 'xyz';
    
    % Remove the nan 
    if nargin < 2 || ignoreNaN
        RT = RT(:,:,~isnan(RT(1,1,:)));
    end
    
    % The mean of the RT is the target of the optimization
    RT = mean(RT, 3);
    
    % Deal with rototranslation matrix (4x4 with translations on 4th col)
    RTmean = RT; % If exists, the average of the 4th column is just the mean of it
    RT = RT(1:3, 1:3); % Remove the 4th column if it exists
    
    % Convert all the matrices to cardan angles
    A = fromMatrixToAngle(RT(1:3, 1:3), seq);
    
    % Initial guess of the optimization
    x0 = mean(A,2); 

    % Optimise
    opt = optimoptions('lsqnonlin', 'display', 'off');
    anglesMean = lsqnonlin(@obj, x0, [], [], opt);

    % Put the answer into matrix
    RTmean(1:3, 1:3) = fromAngleToMatrix(anglesMean, seq);
    
    % Objective function
    function err = obj(x)
        % Convert the angles into rotation and adjust the size
        rt = fromAngleToMatrix(x, seq);
        
        % Compute the error 
        err = rt - RT;
        err = err(:);
    end
    
end
