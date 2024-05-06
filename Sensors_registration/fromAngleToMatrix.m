function mat_out = fromAngleToMatrix(angles, seq)
% Returns the matrix of rotation given a particular angle sequence

	allR.Rx = @(a)	[1,0,0;0,cos(a),-sin(a);0,sin(a),cos(a)]; % Definining the X-rotation
	allR.Ry = @(b)	[cos(b),0,sin(b);0,1,0;-sin(b),0,cos(b)]; % Definining the Y-rotation
	allR.Rz = @(c)	[cos(c),-sin(c),0;sin(c),cos(c),0;0,0,1]; % Definining the Z-rotation

	seq = lower(seq); % Mettre tout en minuscule
	nangles = size(angles,1); % Find how many angles there are 
	nframe = size(angles,2);
    
    % Make sure the user defined an angle name for each angles
    if nangles ~= length(seq)
        error('The sequence must correspond to the number of angles');
    end
    
    % Convert in terms of allR names
    Rseq = cell(nangles, 1);
    for i=1:nangles
       Rseq{i} = sprintf('R%s', seq(i));
    end
	
    % Perform the conversion using a string that is going to be evaluated
    mat_out = repmat(eye(3,3),[1, 1, nframe]);
    for j=1:nframe
        for i=1:nangles % For all the rotations
            mat_out(:, :, j) = mat_out(:, :, j) * allR.(Rseq{i})(angles(i, j));
        end
    end
end 
