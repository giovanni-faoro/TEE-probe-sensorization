function [c ceq] = cons_Ch(r)

%% Check input parameter
if (size(r,1) ~= 1 || size(r,2)~= 15)
    error('Incorrect parameter vector ');
end

%% Equality and disequality non linear constraint definition;
R = reshape(r(7:15),3,3);

c = [];
% 
% ceq = R*R' - eye(3);
ceq = reshape(R*R' - eye(3),1,9);
return
% 
