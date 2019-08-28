% demo_Marker

% SimpleMarker is the original class with few properties.
% Marker is used to save the movement of an optical marker.
% MarkersGroupZero is used to algin the markers as a group.

% Haopeng Hu
% 2019.08.27
% All rights reserved

%% Initialization

% % Marker (DO NOT need to init. a SimpleMarker object exclusively)
% % It is recommanded to init. a Marker by XYZ and Time vectors.
% % load('Data\20190622AssemblyData\Demo1Raw.mat');

% demo01marker01 = Marker([X1 Y1 Z1],Time);

% % MarkersGroupZero
% % Init. a markers group by three markers
% % load('Data\20190622AssemblyData\Demo1Raw.mat');

% demo01markgp01 = MarkersGroupZero([demo01mark01 demo01mark02 demo01mark03]);