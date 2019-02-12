function sim=pathPlanningSimulation(fName, shape, rOpts, sOpts)



rng('default');
numRobot = sOpts.numRobot;
dt       = sOpts.dt;
t        = 0:dt:sOpts.T;
Nt       = length(t);


i_max = 500;                     %% max iterations in each segment
eTol = sOpts.eTol;               %% tolerance for total energy;
tol  = rOpts.robotRadius/1000;   %% tolerance for rCh on gradient descent segment 

log = sprintf('Shape Name: %s\n  %6s\t %6s \t %6s \t %6s', fName,'ID segment', 'alpha', 'T', 'Energy');
crash     = [];
robotChange    = 0*t;




%% Set initial position of robots

sp = 6; % initial spread between robots
if rOpts.robotRadius<.05; sp=10; end

switch sOpts.init_type
    case 'corner'
        [gx, gy]    = meshgrid((-sOpts.cbd+sp*rOpts.robotRadius):(sp*rOpts.robotRadius):(sOpts.cbd-sp*rOpts.robotRadius));
        gx = gx(1:ceil(sqrt(numRobot)),1:ceil(sqrt(numRobot)));
        gy = gy(1:ceil(sqrt(numRobot)),1:ceil(sqrt(numRobot)));
        initloc = [gx(1:numRobot); gy(1:numRobot)];
    case 'rand'    
       [gx, gy]    = meshgrid((-sOpts.cbd+sp*rOpts.robotRadius):(sp*rOpts.robotRadius):(sOpts.cbd-sp*rOpts.robotRadius));
        I=randperm(length(gx)^2);
        initloc = [gx(I(1:numRobot)); gy(I(1:numRobot))];
end


%% Path-planning algorithm
eF    = zeros(length(t), numRobot); 
eG    = zeros(length(t), numRobot);
pos   = zeros(2, Nt, numRobot); 
pos(1:2,1,1:numRobot) = initloc;
seg(1).startLoc(1,1:2,1:numRobot) = initloc;
seg(1).endLoc(1,1:2,1:numRobot)   = initloc;


tic;
i = 1; n = 1; mE = inf; 
switch sOpts.sim_type
    case 'GD'
        seg.startTime(n)    = t(i);
        seg.endTime(n)    = t(i);
        seg.numSegments=1;
        i_max = inf;
        [pos, t, i,  robotChange, eF, eG, ~, crash] = GDSegment(rOpts,  shape,t, i, robotChange,  eF, eG,crash, Nt, pos,sOpts, -1, -1,Nt, '1');
        
        
    case 'ID'
        while abs(mE)> eTol || i <100 
            % Part 1: Perform diffusion step to decide temporary location of robots
            T(n)                = min(.9*max(t)-t(i), abs(sOpts.IDTimeScale*(rand)));
            alpha_robot         = sOpts.DiffusionScale*(rand(2,numRobot));
            alpha_id(n)         = mean(alpha_robot(:));
            
            seg.startLoc(n,:,:)             = squeeze(pos(1:2,i,1:numRobot));
            fprintf('Diffusion: %f \t T= %f alpha=%f\n', t(i),T(n), mean(alpha_robot(:)));
            
            [tmploc, tmp_shape] = diffSegment(rOpts, shape, squeeze(seg.startLoc(n,:,:)), sOpts, T(n), alpha_robot);
            seg.endLoc(n,:,:)   = tmploc;
            
            % Part 2: Get robots to "tmploc" using gradient descent. Here the shape
            % function is ignored. The stopping criteria is based on the change in
            % robots.
            
            seg.startTime(n)    = t(i);
            [pos, t, i,  robotChange, eF, eG, ~, crash]=GDSegment(rOpts,  tmp_shape,t, i, robotChange,  eF, eG, crash, .9*Nt, pos,sOpts, tol, eTol, i_max,'2');
            seg.endTime(n)  = t(i); n=n+1;
            
            
            % Part 3: Regular gradient descent to the original objective for the
            % shape
            [pos, t, i,  robotChange, eF, eG, mE, crash]=GDSegment(rOpts,  shape,t, i, robotChange,  eF, eG,crash, .9*Nt, pos,sOpts, tol, eTol,i_max, '1');
            
            if  abs(mE)<eTol || i>=.9*Nt
                
                [pos, t, i,  robotChange, eF, eG, mE, crash] = GDSegment(rOpts,  shape,t, i, robotChange, eF, eG,crash, Nt, pos,sOpts, .01*tol, .01*eTol,50*i_max, 'final');
                
                fprintf('Energy = %d  eTol = %d, iter = %d/%d', mE, eTol, i, Nt);
                seg.numSegments  = n-1;
                break;
                
            end
            
            
            
        end
end
i=i-1;
sim = struct( 'robotChange', robotChange(1,1:i), 'energy', eF(1:i,:) + eG(1:i,:), ...,
    'log', log, 'eF', eF(1:i,:), 'eG', eG(1:i,:), 'crash', crash, ...,
    't',t(1:i), 'pos',pos(:,1:i,:),'seg',seg);



end

function [pos, t, i, robotChange, eF, eG, mE, crash]=GDSegment(rOpts, shape,t, i, robotChange,eF, eG, crash, Nt, pos, sOpts, tol, eTol,i_max, str)
%% Gradient descent
numRobot=sOpts.numRobot;
mE = inf; i_0 = i; rCh = inf;

while (rCh>tol && mE>eTol)   && i < min(Nt, i_0+i_max)
    [dir, mE,eF(i,:), eG(i,:)]      = decideDir(rOpts,squeeze(pos(1:2,i,1:numRobot)), shape);
   
    pos(1:2,i+1,1:numRobot)         = moveRobots(dir, sOpts.dt,0, sOpts.cbd, pos(1:2,i,1:numRobot), rOpts.robotRadius);
    robotChange(i)                  = norm((abs(squeeze(pos(1:2,i,1:numRobot)-pos(1:2,i+1,1:numRobot)))))/sqrt(numRobot);
    rCh                             = robotChange(i);
    minDistance  = min(pdist(squeeze(pos(1:2,i,1:numRobot))'));
    fprintf('GD %s: %f \t %f  tol= %f mean energy = %f\n', str, t(i),robotChange(i), tol, mE);
    
    % If the robots are too close, there is a "crash"
    if minDistance<rOpts.robotRadius*2
        crash       = [crash t(i)];
        disp('crash')
        pause;
    end
    i = i+1;
end





end
function [tmploc, tmp_shape] = diffSegment(rOpts, shape, startLoc, sOpts, T, alpha_robot)
%% Gradient descent + diffusion 

tmploc = startLoc;
% Run the diffusion segment to obtain the temporary target location
for j=1:round(T/sOpts.dt)
    dir = decideDir(rOpts,tmploc, shape);
    tmploc = moveRobots(dir, sOpts.dt,alpha_robot, sOpts.cbd, tmploc, rOpts.robotRadius);
end
endLoc=tmploc;

% Potential fields corresponding to the temporary destinations
dfun = mean(arrayfun(@(K) max(norm(endLoc(:,K)-startLoc(:,K)), rOpts.robotRadius), 1:sOpts.numRobot));
endLoc = squeeze(tmploc); tmp_shape.bd=sOpts.cbd;
tmp_shape.shapef = @(x,y) shape.shapef(startLoc(1,:),startLoc(2,:));
tmp_shape.shapefx=@(x,y) exp(-((x-endLoc(1,:)).^2+(y-endLoc(2,:)).^2)./(2*dfun.^2)).*(x-endLoc(1,:))./(2*dfun.^2);
tmp_shape.shapefy=@(x,y) exp(-((x-endLoc(1,:)).^2+(y-endLoc(2,:)).^2)./(2*dfun.^2)).*(y-endLoc(2,:))./(2*dfun.^2);

end





