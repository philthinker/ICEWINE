classdef MarkersDemo
    %MarkersDemo A dual-arm assembly demo. 
    %   Note that only TWO components are supported and they must share the same time series.
    %   You can init. the demo by a series of markers groups.
    %   componentFixed: 1 x D or D x 1 MarkersGroup, the fixed component
    %   componentFlying: 1 x D or D x 1 MarkersGroup, the component to be
    %   assembled to componentFixed
    
    properties (Access = public)
        componentFixed;     % Fixed component
        componentFlying;    % Flying component
        Time;               % Time serieses 
    end
    
    properties (Access = protected)
        D;                  % Num. of demos
    end
    
    methods
        function obj = MarkersDemo(componentFixed,componentFlying)
            %MarkersDemo Init. the demo by two markers groups or components
            %   componentFixed: 1 x D or D x 1 MarkersGroup, the fixed component
            %   componentFlying: 1 x D or D x 1 MarkersGroup, the component
            %   to be assembled to componentFixed
            obj.componentFixed = componentFixed;
            obj.componentFlying = componentFlying;
            obj.D = min([length(componentFixed),length(componentFlying)]);
            obj.Time = cell(1,obj.D);
            for i = 1:D
                obj.Time{i} = componentFixed.getTime();
            end
        end
    end
    
    methods (Access = public)
        function obj = singleArmMap(obj)
            %singleArmMap Map the componentFlying to its relative
            %coordinates.
            %   Note that the componentFixed will be set to the 'origin'
        end
        function obj = decay(obj,N)
            %decay Transform the time series to its decay representation
            %   N: Num. of decayed frames (optional). Note that once an N
            %   is assigned, the markers group will be resampled.
            obj = obj.Property1 + inputArg;
        end
    end
end

