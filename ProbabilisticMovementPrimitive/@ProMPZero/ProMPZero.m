classdef ProMPZero
    %ProMPZero Probabilistic Movement Primitive
    %   For more details, refer to A. Paraschos "Using Probabilistic
    %   Movement Primitives in Robotics". Autonomous Robots, 2018.
    %
    %   Haopeng Hu
    %   2020.01.01 Happy New Year!
    %   All rights reserved
    
    properties
        Property1
    end
    
    methods
        function obj = ProMPZero(inputArg1,inputArg2)
            %ProMPZero ��������ʵ��
            %   �˴���ʾ��ϸ˵��
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 �˴���ʾ�йش˷�����ժҪ
            %   �˴���ʾ��ϸ˵��
            outputArg = obj.Property1 + inputArg;
        end
    end
end

