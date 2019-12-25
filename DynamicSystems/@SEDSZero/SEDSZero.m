classdef SEDSZero
    %SEDSZero Stable estimator of dynamic systems
    %   此处显示详细说明
    
    properties
        Property1
    end
    
    methods
        function obj = SEDSZero(inputArg1,inputArg2)
            %SEDSZero 构造此类的实例
            %   此处显示详细说明
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

