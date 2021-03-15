function [gapIDs, logiIDs, gap] = setCuttingPoints(trajIn, c1,c2, minInterval)
%setCuttingPoints Set the cutting point of one trajectory
%   trajIn: N x D, the original trajectory.
%   c1: Integer, the 1st cutting point.
%   c2: Integer, the 2nd cutting point.
%   minInterval: Integer, the minimum gap length.
%   -------------------------------------------------
%   gapIDs: NC x 1, the IDs of the cutted data.
%   logiIDs: N x 1 boolean, the logic IDs of the trajectory in which the
%   cutted ones are false.
%   gap: NC x D, the gap data.

N = size(trajIn,1);
c1 = max([1,round(c1(1,1))]);
c2 = round(c2(1,1));
minInterval = max([1, round(minInterval(1,1))]);
minInterval = min([N, minInterval]);

if c1 <= c2
    if c2-c1<minInterval
        c2 = c1+minInterval-1;
        if c2 > N
            c2 = N;
            c1 = N-minInterval+1;
        end
    end
else
    c2 = c1+minInterval-1;
    if c2 > N
        c2 = N;
        c1 = N-minInterval+1;
    end
end

gapIDs = (c1:c2)';
logiIDs = true(N,1);
logiIDs(gapIDs) = false;
gap = trajIn(gapIDs,:);

end

