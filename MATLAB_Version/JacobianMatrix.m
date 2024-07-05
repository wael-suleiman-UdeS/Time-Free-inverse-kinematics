%Copyright, Wael Suleiman, 2016
function Jp=JacobianMatrix(theta,P)

Z0=[0 0 1]';

% Jacobian matrix
J=[];

for k=1:length(theta),
	Jc = [cross(Z0,(P(end).p -P(k).p)); Z0]; % column k of Jacobian matrix 
	J=[J, Jc];
end;

Jp=J(1:2,1:end);


return
