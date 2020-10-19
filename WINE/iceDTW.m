function [trajsOut,r] = iceDTW(trajs,window)
%iceDTW Dynamic time warping
%   trajs: 1 x M cell of N x D trajectories
%   window: scalar, the width of warping time window
%   -------------------------------------------------
%   trajsOut: 1 x M cell of N x D trajectories
%   r: 1 x M struct, memory

% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

nbData = 2;                 % Length of each trajectory
nbSamples = length(trajs);  % Number of demonstrations
for i = 1:nbSamples
    nbData = max(nbData,size(trajs{i},1));
end
nbVar = size(trajs{1},2);   %Number of dimensions

% Resampling
s = cell(1,nbSamples);
for n=1:nbSamples
    s{n} = pchip(1:size(trajs{n},1), (trajs{n}(:,1:nbVar))', linspace(1,size(trajs{n},1),nbData));
end

% Memory allocation
r.Data = zeros(nbVar,nbData);
r.wPath = zeros(2,nbData);
r = repmat(r,[1,nbSamples]);

% Loop
r(1).Data = s{1};
for n=2:nbSamples
% 	[r(1).Data, r(n).Data, r(n-1).wPath] = DTW(r(1).Data, s{n}, window);
%    function [x2, y2, p] = DTW(x, y, w)
    x = r(1).Data;
    y = s{n};
    %% DTW
    sx = size(x,2);
    sy = size(y,2);
    window = max(window,abs(sx-sy)); 
    
    %Initialization
    D = ones(sx+1,sy+1) * Inf;
    D(1,1) = 0;
    
    %DP loop
    for i=1:sx
        for j=max(i-window,1):min(i+window,sy)
            D(i+1,j+1) = norm(x(:,i)-y(:,j)) + min([D(i,j+1), D(i+1,j), D(i,j)]);
        end
    end
    
    i=sx+1; j=sy+1;
    p=[];
    while i>1 && j>1
        [~,id] = min([D(i,j-1), D(i-1,j), D(i-1,j-1)]);
        if id==1
            j=j-1;
        elseif id==2
            i=i-1;
        else
            i=i-1;
            j=j-1;
        end
        p = [p [i;j]];
    end
    
    p = fliplr(p(:,1:end-1)-1);
    
    x2 = x(:,p(1,:));
    y2 = y(:,p(2,:));
    
    %Resampling
    x2 = spline(1:size(x2,2), x2, linspace(1,size(x2,2),sx));
    y2 = spline(1:size(y2,2), y2, linspace(1,size(y2,2),sx));
    %% DTW Output
    r(1).Data = x2;
    r(n).Data = y2;
    r(n-1).wPath = p;
    
	%Realign previous trajectories
	p = r(n-1).wPath(1,:);
	for m=2:n-1
		DataTmp = r(m).Data(:,p);
		r(m).Data = pchip(1:size(DataTmp,2), DataTmp, linspace(1,size(DataTmp,2),nbData)); %Resampling
	end
end

trajsOut = cell(1,nbSamples);
for i = 1:nbSamples
    trajsOut{i} = (r(i).Data)';
end

end