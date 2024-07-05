function dqt  = IKSolver_basic(Jp, xdot,VeloMax)
n=size(Jp,2);
m=size(Jp,1);
Aopt= Jp;

Bopt= xdot;
Q=1e3*eye(n);


A_ub = VeloMax*ones(n,1); %Upper limits
A_lb = -A_ub; % Lower limits

F=zeros(n,1);

options=optimset('Algorithm', 'active-set'); % Active-set optimization
%method can be used

[u,FVAL,EXITFLAG]  = quadprog(Q, F, [],[], Aopt, Bopt, A_lb, A_ub, zeros(n,1), options);

switch (EXITFLAG)
    case 0
        fprintf ("Maximum number of iterations reached.\n");

    case -2
        fprintf ("The problem is infeasible.\n");

    case -3
        fprintf ("The problem is not convex and unbounded.\n");

    case 1
        fprintf ("Global solution found.\n");

    case 4
        fprintf ("Local solution found.\n");

end

dqt=u;



