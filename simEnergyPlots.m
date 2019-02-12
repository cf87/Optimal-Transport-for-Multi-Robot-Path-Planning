
%% Plots
example = 'Q-corner';
switch example
    case 'Jie-rand'
        numRobot = 400;
        robotRadius = .05;
        clims=[0, .1];
    case 'Q-corner'
        numRobot = 150;
        robotRadius = .05;
          clims=[0, .1];
end
ext = sprintf('r-%.3f-n-%d',robotRadius, numRobot);

load(sprintf('%s-ID-%s.mat', example, ext));
simID=sim;
load(sprintf('%s-GD-%s.mat', example, ext));
simGD = sim;

%% Start and end positions for ID and GD
r=robotRadius;
bd= shape.bd;
cbd=6;



figure; set(gcf,'OuterPosition', [163 507 901 291]);
subplot(1,3,1)
h=plot(shape.A(1,:),shape.A(2,:), 'k.'); set(h, 'color',[0.650980392156863 0.650980392156863 0.650980392156863])
hold all; title('Starting locations');currentunits = get(gca,'Units');
set(gca,'cLim',clims);

set(gca, 'Units', 'Points'); axpos = get(gca,'Position');
set(gca, 'Units', currentunits); markerWidth = r*axpos(3)/diff(xlim);
h=scatter(simID.pos(1,1,1:numRobot), simID.pos(2,1,1:numRobot),20,simGD.eF(1,1:numRobot),'filled');
colormap jet;

subplot(1,3,2)
hold all;
scatter(simGD.pos(1,end,1:numRobot), simGD.pos(2,end,1:numRobot),20,simGD.eF(end-1,1:numRobot),'filled');
title('Final position - Gradient Descent');
axis([-shape.bd, shape.bd, -shape.bd, shape.bd]);

subplot(1,3,3)
hold all;
h=scatter(simID.pos(1,end,1:numRobot), simID.pos(2,end,1:numRobot),20,simID.eF(end-1,1:numRobot),'filled');
title('Final position - ID based Descent');
set(gca,'cLim',clims)
colorbar('Position',[.92 .11 .02 .8132]);
axis([-shape.bd, shape.bd, -shape.bd, shape.bd]);

%% ID simulation snapshots


[~, Nt, e]=size(simID.pos);
it=[1 round(Nt/16) round(Nt/8) round(Nt/4) round(Nt/2) Nt];

theta = 0:.1:2*pi;
figure; set(gcf,'Position', [163 302 919 423]);

shape.bd=5;
subplot(2,3,1)
h=plot(shape.A(1,:),shape.A(2,:), 'k.'); set(h, 'color',[0.650980392156863 0.650980392156863 0.650980392156863]);
colormap jet
axis([-cbd cbd -cbd cbd])
title('Initial Configuration')


hold all;
for j=1:6
    loc_ID=squeeze(simID.pos(:,it(j),:));
    x0=loc_ID(1,:);
    y0=loc_ID(2,:);
    hold all;
    sp(j)=subplot(2,3,j); hold all;
    iter=it(j);
    en=simID.eF(it(j),:);
    scatter(x0', y0',20,en','filled');
    title(sprintf('%d iterations', it(j)));
    axis([-shape.bd, shape.bd, -shape.bd, shape.bd]);
    colormap jet;
    set(gca,'cLim', clims)
    colorbar off;
    
end

subplot(2,3,1)
title('Initial Configuration')

axis([-cbd cbd -cbd cbd])
colorbar('Position',[.92 .11 .02 .8132]);



%% Plot the energy vs iteration

[N ~] = size(simID.eF);
N=N-1;
GD_en=sum(abs(simGD.eF(1:N,:)+simGD.eG(1:N,:)),2)/numRobot;
ID_en=sum(abs(simID.eF(1:N,:)+simID.eG(1:N,:)),2)/numRobot;

figure;
plot((GD_en) ,'linewidth',2);
hold all;
plot((ID_en) ,'linewidth',2);
xlabel('Iteration');
ylabel('\psi(X)');
legend('Gradient Descent', 'Intermittent Diffusion')
title(sprintf(' "%s" potential function: r=%.4f N=%d',example, r, numRobot ))


%% Plot repelling function G
figure;
G_GD=sum(abs(simGD.eG(1:N,:)),2)/numRobot;
G_ID=sum(abs(simID.eG(1:N,:)),2)/numRobot;

figure;
loglog(simGD.t(1:N), ((G_GD)), simID.t(1:N), ((G_ID)),'linewidth',2)
xlabel('t');
ylabel('log(G(X_t))');
legend('Gradient Descent', 'Intermittent Diffusion','Location','eastoutside')
title(sprintf(' "%s" repel function: r=%.2f N=%d',shapeName, r, numRobot ))

%% Plot shape density function F
figure;
F_GD=sum(abs(simGD.eF(1:N,:)),2)/numRobot;
F_ID=sum(abs(simID.eF(1:N,:)),2)/numRobot;

plot((1:N), log((F_GD)), (1:N), log((F_ID)),'linewidth',2)
xlabel('t');
ylabel('log(F(X_t))');
legend('Gradient Descent', 'Intermittent Diffusion')
title(sprintf(' "%s" shape density function: r=%.2f N=%d',shapeName, r, numRobot ))



%% Plot the individual trajectories side-by-side
close
energy_GD=abs(simGD.energy);
energy_ID=abs(simID.energy);

figure;
set(gcf,'OuterPosition',[92 527 1283 344])
subplot(1,2,1)
plot(simGD.t, (energy_GD))
title('Energy Profile: Gradient Descent');
ylabel('log(|\Psi(X)|)')

subplot(1,2,2)
plot(simID.t,(energy_ID))
title('Energy Profile: ID based Descent')
ylabel('log(|\Psi(X)|)')


%% Shape function alone

[X, Y] = meshgrid(-shape.bd:.01:shape.bd);
[Xnew, Ynew] = meshgrid(-shape.bd:.01:shape.bd);
figure; hold all;
F = interp2(X,Y,shape.shapef(X,Y), Xnew,Ynew, 'spline') ;
contour(Xnew,Ynew,-F,[40]); colormap jet; colorbar;

%% Plot a couple of the trajectories of the robots from their starting positions to their final destinations
figure;
h=plot(shape.A(1,:),shape.A(2,:), 'k.'); set(h, 'color',[0.850980392156863 0.850980392156863 0.850980392156863])
hold all;
X=squeeze(simID.pos(1,:,[100:105 6 50]));
Y= squeeze(simID.pos(2,:,[100:105 6 50]));
plot(X,Y,'linewidth',2)





