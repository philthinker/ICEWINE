function [h, msh] = plot2DArrowSC(pos, dir, col, lnw, sz, alpha)
% Simple display of a 2D arrow
%
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

if nargin<6
	alpha = 1;
end
if nargin<5
	sz = norm(dir) .* 4E-2;
end
if nargin<4
	lnw = 2;
end
if nargin<3
	col = [0,0,0];
end

msh = pos;
pos = pos+dir;
h = 0;
if norm(dir)>sz
  d = dir/norm(dir);
  prp = [d(2); -d(1)];
  d = d*sz;
  prp = prp*sz;
  msh = [msh, pos-d, pos-d-prp/2, pos, pos-d+prp/2, pos-d, msh];
  h = patch(msh(1,:), msh(2,:), col, 'edgecolor',col,'linewidth',lnw,'edgealpha',alpha,'facealpha',alpha); 
end