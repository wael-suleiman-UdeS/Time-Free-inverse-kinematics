%Copyright, Wael Suleiman, 2016
function InverseKinematicsControl(Pd,dt,VeloMax, method, plot_skip,new_fig)

  global theta
  global l
  global theta_traj
  global cart_traj  

  if nargin()<4,
  plot_skip=3;
  new_fig=false; 
  end;

  
Pe=fd(theta,l); 
Jp=JacobianMatrix(theta,Pe);

if new_fig,
  figure,
end;

e=Pd-Pe(end).p(1:2);

xdot = e/dt;

switch lower(method)
        case {'conventional'}
        
            Z= IKSolver_basic(Jp, xdot,VeloMax);
        case {'conventional_relaxed'}
        
            Z= IKSolver_basic_relaxed(Jp, xdot, VeloMax);
end;

dtheta= Z*dt;

% Mechanism update
theta=theta+dtheta; 
Pe=fd(theta,l); 
cart_traj=[cart_traj Pe(end).p]; 


theta_traj=[theta_traj theta];

if (mod(length(theta_traj),plot_skip)==0),
 Plot_robot(Pe,true);
end



return
