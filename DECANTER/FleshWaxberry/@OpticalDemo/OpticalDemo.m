classdef OpticalDemo
    %OpticalDemo Demos of optical motion capture devices
    %  You can take both the left and right marker as base marker.
    
    properties
        Marker1s;        % Marker 1s (left)
        Marker2s;        % Marker 2s (right)
        M;                   % Num. of demo
    end
    
    methods
        function obj = OpticalDemo(Marker1s,Marker2s)
            %OpticalDemo Init. with two markers
            %   Marker1, Marker2: 1 x M, @OpticalMarker array
            obj.Marker1s = Marker1s;
            obj.Marker2s = Marker2s;
            obj.M = min([length(obj.Marker1s), length(obj.Marker2s)]);
        end
    end
    
    methods (Access = public)
        function [positions] = getRelativePosition(obj,right)
            %getRelativePosition Get the relative position by taking one
            %marker as base marker (default: left marker)
            %   positions: 1 x M cell, the relative positions
            %   right: boolean, true for taking the right marker as base
            %   marker, false for left (default:false)
            if nargin < 2
                right = false;
            end
            positions = cell(1,obj.M);
            for i = 1:obj.M
                if right
                    % Right marker as base
                    positions{i} = obj.Marker1s(i).positions - obj.Marker2s(i).positions;
                else
                    % Left marker as base
                    positions{i} = obj.Marker2s(i).positions - obj.Marker1s(i).positions;
                end
            end
        end
        
    end
    
    methods (Access = public)
        % Figure
        function [] = plot3Markers(obj)
            %plot3Markers Plot the markers
            figure; hold on;
            for i = 1:obj.M
                % Left
                marker1 = obj.Marker1s(i).positions;
                plot3(marker1(:,1), marker1(:,2), marker1(:,3), 'Color', [0.9290 0.6940 0.1250]);
                % Right
                marker2 = obj.Marker2s(i).positions;
                plot3(marker2(:,1), marker2(:,2), marker2(:,3), 'Color', [0 0.4470 0.7410]);
            end
            grid on; axis equal;
            view(3);
        end
    end
end

