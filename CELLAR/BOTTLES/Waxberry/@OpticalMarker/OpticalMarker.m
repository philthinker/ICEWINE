classdef OpticalMarker
    %OpticalMarker Store position of optical markers in one rigid body
    %   We always assume that the data are formulated as [x,y,z]
    
    properties
        positions;          % Positions of the center
        raw;                  % Raw data
        SO3s;                % Orientations of the data 
        num;                 % Num. of markers
        N;                      % Num. of data points
    end
    
    methods
        function obj = OpticalMarker(data,m)
            %OpticalMarker Init. the marker
            %   data: N x 3 * m, position data
            %   m: integer, number of markers in one rigid body
            obj.num = round(m);
            obj.raw = data(:,1:3*obj.num);
            obj.N = size(obj.raw,1);
            obj.positions = zeros(obj.N,3);
            for i = 1:3
                for j = 1:obj.num
                    obj.positions(:,i) = obj.positions(:,i) + obj.raw(:,(i-1)+(j-1)*3+1);
                end
                obj.positions(:,i) = obj.positions(:,i)/obj.num;
            end
            obj.SO3s = repmat(eye(3),[1,1,obj.N]);
        end
        
        function [] = plot3(obj,plus)
            %plot Plot the trajectories
            %   plus: boolean, true for plot the markers together with
            %   center (default:false)
            if nargin < 2
                plus = false;
            end
            figure;
            plot3(obj.positions(:,1),obj.positions(:,2),obj.positions(:,3));
            grid on; axis equal;
            view(3);
            xlabel('x'); ylabel('y'); zlabel('z');
            if plus
                hold on;
                for i = 1:obj.num
                    plot3(obj.raw(:,(i-1)*3+1),obj.raw(:,(i-1)*3+2),obj.raw(:,(i-1)*3+3),'Color',[0.6,0.6,0.6]);
                end
            end
        end
        
    end
end

