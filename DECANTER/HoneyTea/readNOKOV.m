function [Data,N] = readNOKOV(nokovTS,fps,VelAcc)
%readNOKOV Read NOKOV's table data
%   nokovTS: table, NOKOV's table data
%   fps: integer, fps
%   VelAcc: boolean, velocity and acceleration required (default: false)
%   Data: struct, the NOKOV's struct data
%   N: num. of frames

if nargin < 3
    VelAcc = false;
end

N = size(nokovTS,1);
Data.fps = fps;
Data.time = nokovTS.Time;
Data.position = [nokovTS.p,nokovTS.VarName4,nokovTS.VarName5];
if VelAcc
    Data.velocity = [nokovTS.VarName6, nokovTS.VarName7, nokovTS.VarName8, nokovTS.VarName9];
    Data.acceleration = [nokovTS.VarName10, nokovTS.VarName11, nokovTS.VarName12, nokovTS.VarName13];
end

end

