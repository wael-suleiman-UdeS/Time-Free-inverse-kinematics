function dqt  = IKSolver_basic_relaxed(Jp, xdot, VeloMax)
   
n=size(Jp,2);
m=size(Jp,1);
Aopt= [Jp, eye(m)];

Bopt= xdot;
Q=1e1*eye(n+m);

for i=n+1:n+m, % slack variables weighting    
   Q(i,i)=1e4;
end;


A_ub = [VeloMax*ones(n,1);1e3*ones(m,1)]; %Upper limits
A_lb = -A_ub; % Lower limits
     
F=zeros(n+m,1); 

options=optimset('Algorithm', 'active-set'); % Active-set optimization
%method can be used 

 [u,FVAL,EXITFLAG]  = quadprog(Q, F, [],[], Aopt, Bopt, A_lb, A_ub, [], options);
 
 switch (EXITFLAG)
  case 0
printf ("Maximum number of iterations reached.\n");

  case -2
printf ("The problem is infeasible.\n");

  case -3
 printf ("The problem is not convex and unbounded.\n");

  case 1
  printf ("Global solution found.\n");

  case 4
  printf ("Local solution found.\n");
  
endswitch

 dqt=u(1:n); 
     





