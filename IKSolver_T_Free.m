function Z  = IKSolver_T_Free(Jp, dx, dt, VeloMax),

global theta

   
n=size(Jp,2); 
m=size(Jp,1); 
Aopt= [Jp, zeros(m,1)] ;

Bopt= dx;

Beta=1;

H=1000*eye(n+1); 
H(end,end)=Beta;


Max= 1e7; 
A_ub = Max*ones(n+1,1);

Myeps=dt;

A_lb = [-Max*ones(n,1); Myeps];
A_in1=  [eye(n), -VeloMax*ones(n,1)];
A_in2=  [-eye(n), -VeloMax*ones(n,1)];
A_in = [A_in1; A_in2];
B_in=[zeros(2*n,1)];

x0=[zeros(n,1);dt];  
F=zeros(size(x0));
options=optimset('Algorithm', 'active-set');

[u_ext,FVAL,EXITFLAG]  = quadprog(H, F, A_in,B_in, Aopt, Bopt, A_lb, A_ub,x0,options);

Z=u_ext; 

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



