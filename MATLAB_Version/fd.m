%Forward kinematics 
%Copyright, Wael Suleiman, 2016
function P=fd(theta,l)

P(1).p= [0 0 0]';

theta_k=0;

for k=1:length(theta),

        theta_k=theta(k)+theta_k;
        P(k+1).p = P(k).p +l(k)*[ cos(theta_k)  sin(theta_k) 0]';
        
end;

return 
