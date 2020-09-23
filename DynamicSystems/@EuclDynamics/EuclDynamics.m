classdef EuclDynamics
    %EuclDynamics Typical high order dynamic system with Euclidean
    %distance metric
    %   Iterative and batch usages are supported
    %   dX = V
    %   dV = A
    %   dA = J
    
    properties
        DD;       % Integer, Dim. of data
        DO;       % Integer, Dim. of order
        dt;         % scalar, time step
    end
    
    methods
        function obj = EuclDynamics(DimData,Order,dt)
            %EuclDynamics Initialize with DD, DO, dt
            %   DimData: Integer, Dim. of data
            %   Order: Integer, Dim. of order
            %   dt: scalar, time step
            obj.DD = max(round(abs(DimData)),1);   % Larger than 1
            obj.DO = max(round(abs(Order)),1);       % Larger than 1
            obj.DO = min(5,obj.DO);                         % Lower than 
            obj.dt = max(abs(dt), 1e-3);                    % Larger tthan 1e-3
        end
        
        function output = runItera(obj,input)
            %runItera Run the dynamics iteratively
            %   input: DD x DO, input [p0 v0 a0 j0 ...]
            %   -----------------------------------------
            %   output: DD x DO, output [p1 v1 a1 j1=j0 ...]
            if size(input,1) < obj.DD
                output = zeros(obj.DD,1);
                return;
            end
            input = input(1:obj.DD,:);
            if size(input,2) < obj.DO
                output = input;
                return;
            end
            % [ p1 v1 a1 j1] = [p0 v0 a0 j0] + dt * [v0, a0, j0, 0]
            output = input + obj.dt * [input(:,2,end),zeros(size(input,1),1)];
        end
        
        function output = runBatch(obj,input,zeroState)
            %runBatch Run the dynamics in a batch style
            %   input: DD x N, input [j0 j1 j2 ...]
            %   zeroState: DD x DO-1, current state [p0,v0,a0,...]
            %   -----------------------------------------
            %   output: DD x DO x N, output [pN vN aN jN] .....
            if size(input,1) < obj.DD || size(zeroState,1) < obj.DD
                output = zeros(obj.DD,obj.DO);
                return;
            end
            if size(zeroState,2) < obj.DO-1
                output = zeros(obj.DD,obj.DO);
                return;
            end
            input = input(1:obj.DD,:);
            zeroState = zeroState(1:obj.DD, 1:obj.DO-1);
            N = size(input,2);
            output = zeros(obj.DD,obj.DO,N);
            counter = 1;
            output(:,end,1) = input(:,1);
            output(:,1:obj.DO-1,1) = zeroState;
            while counter <= N
                counter = counter + 1;
                output(:,end,counter-1) = input(:,counter-1);
                output(:,:,counter) = output(:,:,counter-1) + obj.dt * [output(:,2:end,counter-1),zeros(obj.DD,1)];
            end
        end
    end
    
    methods (Access = public)
        % Auxiliary functions
        function output = extractOrders(obj, Data, orders)
            %extractOrders Extract orders of data
            %   Data: DD x DO x N, dynamics data
            %   orders: 1 x K, K <= DO orders to be extracted
            if obj.illegalCheck(Data)
                output = zeros(obj.DD,obj.DO);
                return;
            end
            orders = round(orders(1,:));
            if max(orders) > obj.DO || min(orders) < 1
                output = NaN;
                return;
            end
            output = permute(Data(:,orders,:), [1,3,2]);
        end
        
        function [] = plotAll(obj,Data)
            %plotAll Plot all the data
            %   Data: DD x DO x N, data
            if obj.illegalCheck(Data)
                return;
            end
            N = size(Data,3);
            ylabels = {'Pos.','Vel.','Acc.','Jer.','Jou.'};
            Data = obj.extractOrders(Data,(1:obj.DO));
            t = (1:N) * obj.dt - obj.dt;
            for i = 1:obj.DO
                subplot(obj.DO,1,i);
                for j = 1:obj.DD
                    plot(t,Data(j,:,i));
                    hold on;
                end
                ylabel(ylabels{i});
                grid on;
            end
            xlabel('t');
        end
    end
    
    methods (Access = protected)
        % Data check
        function flag = illegalCheck(obj, Data)
            %illegalCheck Check if the data is illegal
            %   Data: DD x DO x N, data
            %   -----------------------------------------
            %   flag: boolean, true for illegal data detected
            Dim = size(Data,1);
            Ord = size(Data,2);
            flag = ~(( Dim == obj.DD ) && ( Ord == obj.DO ));
        end
    end
end

