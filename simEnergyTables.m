example='Jie-rand';
%% Energy Tables in paper
switch example
    case 'Q-rand'
        load Q-hand
        robotRadius = [.01 .05 .1];
        numRobot = [1000 150 50];
        init_type   = 'rand';
        shapeName='Q';
    case 'Jie-rand'
        load Jie
        robotRadius = [.01 .05 .1];
        numRobot = [3000 400 200];
        init_type   = 'rand';
        shapeName='Jie';
     
end
%%
format long
for j=1:3
    ext = sprintf('r-%.3f-n-%d',robotRadius(j), numRobot(j));
    sim_typeList = {'ID', 'GD'};
    for st=1:2
 %%
        sim_type = sim_typeList{st};
        load(sprintf('%s-%s-%s-%s.mat', shapeName,init_type,sim_type, ext));
        F=sum(abs(sim.eF(1:end-1,:)),2);
        G=sum(abs(sim.eG(1:end-1,:)),2);
        disp(sprintf('N=%d, Iterations=%d, %s %s %s Psi=:%1.8f', numRobot(j),...,
            length(sim.t), shapeName, sim_type, init_type, sum((F(end-1,:)+G(end-1,:))/numRobot(j))))
        
        
        
        
    end
end
