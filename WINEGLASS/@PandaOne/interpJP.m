function [trajOut, NOut] = interpJP(obj,trajIn)
%interpJP Joint position interpolation
%   trajIn: N x 7, joint position trajectory
%   trajOut: NOut x 7, joint position trajectory
%   NOut: integer, number of data in trajOut
%   @PandaOne
%
%   We provide several motion planning methods.
%   Uncomment what you want.
%   Note we ought to always assume that the initial
%   and final velocity and acceleration are ZERO.

N = size(trajIn,1);
trajOut = [];
% q_max, q_min, dq_max, ddq_max, dddq_max
constraint = obj.JointConstraint;

%% Simple point-to-point max jerk iterative interpolation

for i = 2:N
    
end

NOut = size(trajOut,1);

end

