function InverseKinematicsControl_T_Free(Pd,dt,count, VeloMax,method, plot_skip,new_fig)

  global theta
  global l
  global Time
  global Traj_time
  global theta_traj
  global cart_traj  

  if nargin()<3,
  plot_skip=3;
  end;

  
Pe=fd(theta,l); 
Jp=JacobianMatrix(theta,Pe);

if new_fig,
  figure,
end;

if (mod(count,plot_skip)==0),
 Plot_robot(Pe,true);
end

dx=Pd-Pe(end).p(1:2);
    
Z= IKSolver_T_Free(Jp, dx, dt, VeloMax);


dtheta= Z(1:end-1);
T=Z(end); 

newElement.traj=theta;
switch lower(method)
        case {'continuous'}
          theta= theta + dtheta;
          Time=Time+T;
          newElement.time=Time;
          newElement.traj=theta;
      case {'discrete'}
          nTs=round(T/dt)*dt;
          Time=Time+nTs;
          theta= theta + (dtheta/T)*nTs;
          newElement.time=Time;
          newElement.traj=theta;
end;


Traj_time=[Traj_time,newElement];

% Mechanism update
Pe=fd(theta,l); 
cart_traj=[cart_traj Pe(end).p]; 

Jp=JacobianMatrix(theta,Pe);

theta_traj=[theta_traj theta];


return
