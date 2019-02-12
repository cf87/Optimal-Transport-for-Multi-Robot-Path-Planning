
%% Path-planning using optimal transport
% This runs the algorithm described in Frederick et. al, "Multi-robot
% motion planning via optimal transport theory"

example='Q-corner';

%% Choose simulation options
% 'ID': intermittent diffusion 'GD': gradient descent
sim_type = 'GD';



% Choose the desired shape and initial locations and load shape function
% and configuration

switch example
    case 'Q-corner'
        load Q-hand
        robotRadius=.05;
        numRobot = 150;
        eTol         = 3.5e-2;
        shapeName='Q';
        init_type   = 'corner';
        clims=[0 1e-1];
       
    case 'Q-rand'
        load Q-hand
        robotRadius = [.01 .05 .1];
        numRobot = [1000 150 50];
        eTol         = [1e-2 3.5e-2 4.5e-2];
        init_type   = 'rand';
        shapeName='Q';
        clims=[0 1e-1];
    
    case 'Jie-rand'
        load Jie
        robotRadius = [.01 .05 .1];
        numRobot = [3000 400 200];
        eTol         = [.04 .11 .18];
        init_type   = 'rand';
        shapeName='Jie';
        clims=[0 5e-1];
      
end


rOpts=table(robotRadius, numRobot, eTol);

%% Choose the options corresponding to desired radius

for j=1
    numRobot = rOpts.numRobot(j);
    robotRadius = rOpts.robotRadius(j);
    sensorRadius = 10*robotRadius;
    eTol=rOpts.eTol(j);
    
    RobotOpts = struct('G0', .01, 'robotRadius',robotRadius,...,
        'sensorRadius', sensorRadius);
    
    cbd          = 6; % computational boundary size
    dt           = robotRadius/10 ; % time step
    
    switch sim_type
        case 'ID'
            alpha        = robotRadius;
            beta         = 10;
            T            = 500;
            
        case 'GD'
            % Choose T based on the ID T
            if isfile(sprintf('%s-%s-%s-r-%.3f-n-%d.mat', shapeName,...,
                    init_type,'ID', robotRadius, numRobot))
                id_res = load(sprintf('%s-%s-%s-r-%.3f-n-%d.mat', ...,
                    shapeName,init_type,'ID', robotRadius, numRobot));
                T      = id_res.sim.t(end);
                clear id_res;
            else
                T=500;
            end
            
    end
    
    simOpts=struct('dt', dt,'IDTimeScale',beta,'DiffusionScale', alpha,...,
        'init_type',init_type, 'T', T,'t', 0:dt:T, 'eTol',eTol, ...
        'numRobot', numRobot, 'sim_type', sim_type, 'cbd',cbd);
    
    
    %% Run Path Planning Simulation
    
    
     sim = pathPlanningSimulation(shapeName, shape, RobotOpts, simOpts);
     save(sprintf('%s-%s-%s-r-%.3f-n-%d.mat', shapeName,...,
         init_type,sim_type, robotRadius, numRobot), 'sim', 'shape',...,
         'RobotOpts', 'simOpts', '-v7.3')
    
   
    %% Create Movie from simulation
    fname =sprintf('%s-%s-%s-r-%.3f-n-%d.mat', shapeName,init_type,....,
        sim_type, robotRadius, numRobot);
    load(fname);
    
    
    fps = [100 50 50];
    gif = dynamicPlot(sim, simOpts.cbd,shape,  RobotOpts,....,
        fname(1:end-4),clims,fps(j));
    
end
