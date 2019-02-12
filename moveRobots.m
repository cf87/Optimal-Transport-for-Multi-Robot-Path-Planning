function newlocations = moveRobots(dir, dt,alpha, bd, pos, r)

oldlocations = squeeze(pos);
diffusion =  alpha.* randn(size(dir));
dir = bsxfun(@plus , oldlocations, dt*dir);
tmploc = dir+sqrt(dt)*(diffusion).*ones(size(dir));
bd_err = bsxfun(@ge, abs(tmploc), bd);
dir_inside= sign(tmploc).*bsxfun(@mod, abs(tmploc), bd);


if max(max(bd_err))>0
    disp('bd correction');
   disp(sum(bd_err(:)))
end
tmploc = tmploc - 2*dir_inside.*bd_err;

newlocations=tmploc;
 if any(isinf(tmploc(:))) || any(isnan(tmploc(:)))
     disp('err moveRobots')   
     pause; 
        
 end
    
end