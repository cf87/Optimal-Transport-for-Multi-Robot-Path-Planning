function [vec, E, F, G] = decideDir(robotOpts,pos, shape)

X = squeeze(pos);
dir(1,:) = -shape.shapefx(X(1,:),X(2,:));
dir(2,:) = -shape.shapefy(X(1,:),X(2,:));
F = -shape.shapef(X(1,:),X(2,:));

G = 0*X(1,:);
rep = 0*X;


[idx, D]=rangesearch(X', X', robotOpts.sensorRadius, 'Distance', 'euclidean');
R = robotOpts.sensorRadius^2;

for j=1:length(X)
    distX = D{j};
    ind   = idx{j}; ind = ind(ind~=j);
    rij   = distX(2:end).^2*(pi/(2*R));
    
    drij_dx = -(X(1,j)-X(1, ind))*pi/R;
    drij_dy = -(X(2,j)-X(2, ind))*pi/R;
    pot = cot(rij);
    
    dpot_dx = -cot(rij).*csc(rij).*drij_dx;
    dpot_dy = -cot(rij).*csc(rij).*drij_dy;
    
    rep(1,j) = robotOpts.G0*sum(dpot_dx);
    rep(2,j) = robotOpts.G0*sum(dpot_dy);
    if max(robotOpts.G0*pot)>.1
        %  pause;
    end
    G(j) = robotOpts.G0*sum(pot);
    
end

% % % %%
%  [x,y] = meshgrid([-.5:.01:-.01 .01:.01:.5]);
%   R   = 1;
%
% G = @(x,y) cot(pi/2*(x.^2+y.^2)/R);
% GX = @(x,y) -csc(pi/2*(x.^2+y.^2)/R).*cot(pi/2*(x.^2+y.^2)/R);
% surf(x,y,GX(x,y),'linestyle','none')

%%
vec = dir+rep;

E = max(F+G);






plots = 0;
if plots
    %% Helpful plotting script to be used when debugging
    C = fieldnames(shape);
    if strcmp(C{1},'A')
        h=plot(shape.A(1,:),shape.A(2,:), 'k.'); set(h, 'color',[0.650980392156863 0.650980392156863 0.650980392156863])
    end
    hold all;
    en=F+G;
    
    h=scatter(X(1,:)', X(2,:)',20, en','filled'); colorbar
    
    currentunits = get(gca,'Units');
    set(gca, 'Units', 'Points');
    axpos = get(gca,'Position');
    set(gca, 'Units', currentunits);
    markerWidth = robotOpts.robotRadius*axpos(3)/diff(xlim);
    set(h, 'SizeData', pi*markerWidth^2)
    
    quiver(X(1,:),X(2,:), dir(1,:), dir(2,:),'r', 'AutoScale', 'off');
    quiver(X(1,:),X(2,:), rep(1,:), rep(2,:),'b', 'AutoScale', 'off');
    
    axis([-6 6 -6 6]);
end


end