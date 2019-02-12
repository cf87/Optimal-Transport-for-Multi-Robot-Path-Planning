function gif = dynamicPlot(sim, bd,shape, rOpts, name,clims, frameskip)
% Draw the animatation of solution change at time t
% 
% Returns gif, vector of frame, run movie(gcf, gif) to repeat the
% animation, and creates a move file


% movie options
figure(1);
writerObj = VideoWriter(sprintf('%s.mp4', name), 'MPEG-4');
writerObj.Quality = 30;
open(writerObj);



r=rOpts.robotRadius;
seg=sim.seg;
pos=sim.pos;
time=sim.t;
sizeT=length(time);
gif(sizeT) = struct('cdata',[],'colormap',[]);

n=1;

% initial position
figure(1);
plot(shape.A(1,:),shape.A(2,:), 'y.');
hold all;
set(gcf, 'Position', [57 57 943 741]);
set(gca, 'XLim', [-bd bd],'YLim',[ -bd bd]);
axis square;

% get scaling for scatterpoint markers
currentunits = get(gca,'Units');
set(gca, 'Units', 'Points');
axpos = get(gca,'Position');
set(gca, 'Units', currentunits);
markerWidth = r*axpos(3)/diff(xlim);
set(gca, 'XLim', [-bd bd],'YLim',[ -bd bd]);
axis square;

for i = 1 :frameskip:sizeT
    hold off;
    figure(1);
    % plot target shape
    h=plot(shape.A(1,:),shape.A(2,:), 'k.'); 
    set(h, 'color',[0.650980392156863 0.650980392156863 0.650980392156863])
    hold all;
    
    x0=squeeze(pos(1,i,:));
    y0=squeeze(pos(2,i,:));
    en=sim.eF(i,:)'+sim.eG(i,:)';
    h=scatter(x0', y0',1, en','filled');
    set(gca, 'XLim', [-bd bd],'YLim',[ -bd bd]);
    axis square;
    colormap jet;
    
    set(gca,'cLim',clims);
    colorbar;
    set(h, 'SizeData', pi*markerWidth^2)
    hold on;
    
    % the temporary GD segment 
    if time(i) > seg.startTime(n) && time(i)< seg.endTime(n)
        quiver(seg.startLoc(n,1,:), seg.startLoc(n,2,:), seg.endLoc(n,1,:)-seg.startLoc(n,1,:), seg.endLoc(n,2,:)-seg.startLoc(n,2,:),'k--','AutoScale', 'off');
    end
    %
    if time(i)>seg.endTime(n) && n< seg.numSegments
        n=n+1;
    end
    
   title(sprintf('Robot Simulation: %d iterations, Psi(X) = %f, ', i, sum(en)/length(x0)));
   writeVideo(writerObj,getframe(gcf));
end

close(writerObj);

end

