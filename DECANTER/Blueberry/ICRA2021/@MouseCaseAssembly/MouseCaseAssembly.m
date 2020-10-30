classdef MouseCaseAssembly
    %MouseCaseAssembly Mouse case assembly with Panda and Parallel robots
    %   Good luck!
    %   
    %   Haopeng Hu
    %   2020.10.30
    %   All rights reserved
    %   
    %   Notations
    %   |   N:  num. of data
    %   |   M:  num. of demonstration
    
    properties
        label;      % String, the experiment label
    end
    
    methods
        function obj = MouseCaseAssembly(label)
            %MouseCaseAssembly Good luck!
            %   label: String, the experiment label
            obj.label = label;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

