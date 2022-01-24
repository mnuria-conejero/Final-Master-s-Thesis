


%Objective function
fun=@tracklsq3;

%Dimensions (r,g,b,T)
nvars=6;

%Linear inequality constraints A,b
A=[];
b=[];
  
%Linear equality constraints Aeq, beq
Aeq=[];
beq=[];

%Lower bounds
lb=[0,0,0,0,0,0];
%Upper bounds
ub=[1,1,1,1,1,1];

%Nonlinearconstraints
nonlcon=[];

%Integer values (T)
%intcon=[];



%Initial population
populationSize=40;

%Options
options = optimoptions('ga','CrossoverFcn','crossoverscattered');
options = optimoptions(options,'Display','iter');
options = optimoptions(options,'MutationFcn',{@mutationuniform,0.3});
options = optimoptions(options,'SelectionFcn','selectionstochunif');
options = optimoptions(options,'OutputFcn',@gaoutputfcn);

options.CrossoverFraction=0.8;
options.EliteCount=1;
options.PopulationSize=populationSize;
options.FitnessLimit=-0.99;

options.MaxStallGenerations=300;
options.MaxGenerations=500;


%[x,fval,exitflag,output,population,scores] = ga(fun,nvars,A,b,Aeq,beq,lb,ub,nonlcon,intcon,options);
[x,fval,exitflag,output,population,scores] = ga(fun,nvars,A,b,Aeq,beq,lb,ub,nonlcon,options);

%% Final average fitness
hL=x(1)
hH=x(2)
sL=x(3)
sH=x(4)
vL=x(5)
vH=x(6)
fitnessCheck(hL,hH,sL,sH,vL,vH);