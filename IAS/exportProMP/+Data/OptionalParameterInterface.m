classdef OptionalParameterInterface < Common.IASObject
    % This class extends the data manipulation interface and implements 
    % an interface for optional parameters. It allows us to choose which
    % variables are used in the sampling process and which are fixed.
    % Consider for example an object having 2 parameters, 'paramA' and
    % 'paramB'. In some cases, we want to learn 'paramA' and leave 'paramB'
    % fixed. Other cases are vice versa or we even want to learn both
    % parameters. This interface allows us to do so. Also note that this
    % interface requires that the actual classes are *also* subclass from
    % the DataManipulator class.
    %
    % When registering an optional parameter 'Test', four things are happening
    % - the option 'useTest' is registered in the parameter pool. If the value
    % is set to true, the parameter 'Test' is also learned.
    % - the options 'minTest' and 'maxTest' and 'Test' are registered in the 
    %   parameter pool. The min and max Value denote the range of this parameter. 
    % The property 'Test' itself denotes the default value that is used
    % whenever the parameter is not changed ('useTest = 0').
    % - A dynamic property with the name 'Test' is added to the class
    % - If 'useTest' is set to true, 'Test' is also registered as data entry in
    % the data manager (as we want to learn the value for test). In
    % addition, the data entry is added to a dataAlias (for example,
    % 'parameters'). Hence, the data alias can contain the data entry
    % 'Test' (if we want to optimize over it) or not (if we want to use
    % fixed standard value).
    properties
        additionalParameters = {};
        isParametersFromData = true;
    end
    
    methods
        function [obj] = OptionalParameterInterface()   
            obj = obj@Common.IASObject();
        end
       
        
        function [] = registerOptionalParameter(obj, option, defaultGuard, dim, minRange, maxRange, parameterPoolName, level)
            % This function implements the optional parameter interface.
            % Very useful, but not documented yet ;).
            if ( ~exist('minRange','var') )
                minRange = -ones(dim,1);
            end
            
            if ( ~exist('maxRange','var') )
                maxRange = ones(dim,1);
            end
            
            if (~exist('level', 'var'))
                level = 1;
            end
            
            guard = ['use', option];
            
            obj.addSettingsPropAndLink (guard);
            obj.setIfNotEmpty(guard, defaultGuard);
            obj.addSettingsPropAndLink(option);
            
            if ( ~isempty(minRange) )
                obj.addSettingsPropAndLink ( ['min', option], minRange);
            end
            
            if ( ~isempty(maxRange) )
                obj.addSettingsPropAndLink ( ['max', option], maxRange);
            end
            
            if ( obj.(guard) )
                obj.additionalParameters{end+1} = option;
                dataManagerForDepth = obj.getDataManager().getDataManagerForDepth(level);
                dataManagerForDepth.addDataEntry( option, dim, obj.(['min', option]), obj.(['max', option]));
                
                if (exist('parameterPoolName', 'var') && ~isempty(parameterPoolName))
                    dataManagerForDepth.addDataAlias(parameterPoolName, option);
                end
                if (level > 1)
                    obj.getDataManager().finalizeDataManager();
                end
            end
        end
        
        function [] = disableParametersFromData(obj)
            obj.isParametersFromData = false;
        end
        
        function [] = enableParametersFromData(obj)
            obj.isParametersFromData = true;
        end
        
        function [] = inputParameterDeMux(obj, params)
            % Reads in optional parameters and sets them in the object.
            assert( length(params) == length(obj.additionalParameters) );
            if (obj.isParametersFromData)
                for i = 1:length(obj.additionalParameters)
                    obj.(obj.additionalParameters{i}) = params{i};
                end
            end
        end
        
        function [] = registerAdditionalParametersInData(obj, data, suffix, index)
            for i = 1:length(obj.additionalParameters)
                vectorStore = obj.(obj.additionalParameters{i});
                if (size(vectorStore,1) > 1)
                    vectorStore = vectorStore';
                end
                data.setDataEntry([obj.additionalParameters{i}, suffix], vectorStore,index);
            end
        end
        
        
        function [] = addSettingsProp(obj, name)
            f = obj.findprop(name);
            if ( isempty(f) )
                p = obj.addprop(name);
                p.AbortSet = true;
                p.SetObservable = true;
            end
        end
        
        function [] = addSettingsPropAndLink(obj, name, defaultVal)
            obj.addSettingsProp(name);
            if ( exist('defaultVal','var') )
                obj.(name) = defaultVal;
            end
            obj.linkProperty(name);
        end
    end
    
end