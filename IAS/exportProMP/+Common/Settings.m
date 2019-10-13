classdef Settings < dynamicprops;
    % The Settings class implements a parameter pool where we can link
    % properties of several objects. The value of the linked properties
    % will be set to the value set in the parameter pool. If we link a
    % property that is so far not registered in the parameter pool, the
    % property name is registered with the current value of the property
    % set in the parameter pool. Important functions of this class for the
    % user are Settings.registerProperty, Settings.setProperty,
    % Settings.getProperty and Settings.hasProperty. The class also offers
    % the functionality to create several parameter pools (for example if
    % we want to use different parameters for different instances of the
    % same class). However, typically just a single, global parameter pool
    % is used. The global parameter pool can be accessed by calling the
    % empty constructor, i.e., Common.Settings().
    
    properties(SetAccess=public)
        id;
        propInfo = struct();
        
    end
    
    methods
        
        function obj = Settings(id)
            if(exist('id','var'))
                obj.id = lower(id);
            else
                obj.id = Common.SettingsManager.getDefaultId();
            end
            
            if (~strcmp(obj.id, 'new'))
                instance = Common.SettingsManager.getSettings(obj.id);
                if(isempty(instance))
                    Common.SettingsManager.setSettings(obj);
                else
                    obj = instance;
                end
            end
        end
        
        function [] = clean(obj)
            obj.propInfo = struct();
            fNames = fieldnames(obj);
            for i=1:length(fNames)
                if ( ~strcmp(fNames{i},'id') && ~strcmp(fNames{i},'propInfo') )
                    delete(findprop(obj,fNames{i}));
                end
            end
        end
        
        function [] = copyProperties(obj, settings)
            propertyNames = settings.getPropertyNames();
            for i = 1:length(propertyNames)
                obj.setProperty(propertyNames{i}, settings.getProperty(propertyNames{i}));
            end
        end
        
        function [] = removeClients(obj)
            properties = fieldnames(obj.propInfo);
            
            for i = 1:length(properties)
                obj.propInfo.(properties{i}).clients = {};
            end
        end
        
        function [propertyList] = getPropertyNames(obj)
            propertyList = fieldnames(obj.propInfo);
        end
        
        function registerProperty(obj, propName, value)
            if (~exist('value', 'var'))
                value = [];
            end
            if (~isfield(obj.propInfo, propName))
                obj.propInfo.(propName).clients = [];
                obj.propInfo.(propName).clientPropNames = [];
            end
             % create dynamic property
            p = addprop(obj, propName);

            % set initial value if present
            obj.(propName) = value;
           
            p.SetMethod = @set_method;

            % nested getter/setter functions with closure
            function set_method(obj, val)                
                obj.(propName) = val;
                obj.informClients(propName);
            end            
        end
        
        function unregisterProperty(obj,propName)
            if(obj.hasProperty(propName))
                obj.propInfo = rmfield(obj.propInfo,propName);
            end
        end
        
        function printProperties(obj)
            properties = fieldnames(obj.propInfo);
            properties = sort(properties);
            
            for i = 1:length(properties)
                value = obj.(properties{i});
                if (isa(value, 'double'))
                    if (numel(value) == 1)
                        fprintf('%s: %f\n', properties{i}, value);
                    else
                        fprintf('%s: [', properties{i});
                        for i = 1:length(value)
                            fprintf('%f ', value(i));
                        end
                        fprintf(']\n');
                    end
                else
                    if (ischar(value))
                        fprintf('%s: %s\n', properties{i}, value);
                    elseif (isa(value,'function_handle'))
                        fprintf('%s: %s\n', properties{i}, func2str(value));
                    end
                end
            end
        end
        
        
        function linkProperty(obj, client, clientPropName, settingsPropName)
                                                
            if(~obj.hasProperty(settingsPropName))
                obj.registerProperty(settingsPropName, client.(clientPropName));
            else
                client.(clientPropName) = obj.(settingsPropName);
            end
            
            obj.propInfo.(settingsPropName).clients{end+1} = client;                        
            obj.propInfo.(settingsPropName).clientPropNames{end+1} = clientPropName;                        
        end
        
        
        function propValue = getProperty(obj, propName)
            % Returns the value of the property that is registered in the
            % parameter pool.
            propValue = obj.(propName);
        end
        
        function setProperty(obj, propName, value)
            if (obj.hasProperty(propName))
                obj.(propName) = value;
                obj.informClients(propName);
            else
                obj.registerProperty(propName, value);
            end
        end
        
        function setIfEmpty(obj, propName, value)
            if (~obj.hasProperty(propName) || isempty(obj.(propName)))
                obj.setProperty(propName, value);
            end
        end
        
        
        function setProperties(obj, propName, value)
            if (~iscell(propName))
                obj.setProperty(propName, value);
            else
                for i = 1:length(propName)
                    obj.setProperty(propName{i}, value{i});                
                end
            end
        end
        
        
        function [numSettings] = getNumProperties(obj)
            numSettings = length(fieldnames(obj.propInfo));
        end
        
        
        function tf = hasProperty(obj,propName)
            % Returns whether propName is a registered property in the
            % parameter pool.
            tf = isfield(obj.propInfo, propName);
        end
        
        function [] = setToClients(obj)
            
            properties = fieldnames(obj.propInfo);
            
            for i = 1:length(properties)              
                obj.informClients(properties{i});
            end
        end
        
        function [newSettings] = clone(obj, evaluationId)
            % Clones everything except for the clients!
            if (~exist('evaluationId', 'var'))
                evaluationId = 'new';
            end
            newSettings = Common.Settings(evaluationId);
            
            properties = fieldnames(obj.propInfo);
            for i = 1:length(properties)
                newSettings.registerProperty(properties{i}, obj.(properties{i}));
            end
        end
        
        function [hasValue] = hasValue(obj, propertyName, value)
            propertyValue = obj.(propertyName);
            className = class(propertyValue);
            switch className
                case 'double'
                    hasValue = all(propertyValue == value);                    
                case 'char'
                    hasValue = strcmp(propertyValue, value);
                case 'function_handle'    
                    hasValue = strcmp(func2str(propertyValue), func2str(value));
                case 'cell'
                    hasValue = isequal(propertyValue, value);
                case 'logical'
                    hasValue = all(propertyValue == value);      
                otherwise
                    error('Type %s not known for settings\n', className);
            end            
        end
        
        function [sameSettings, differentParameters] = isSameSettings(obj, otherSettings)
            sameSettings = obj.getNumProperties() == otherSettings.getNumProperties();
            differentParameters = {};
            if (sameSettings)
                propertyList = obj.getPropertyNames();
                for i = 1:length(propertyList)
                    sameSettingsLocal = otherSettings.hasValue(propertyList{i}, obj.(propertyList{i}));
                    sameSettings = sameSettings && sameSettingsLocal;
                    if (~sameSettingsLocal)
                        differentParameters{end + 1} = propertyList{i};
                    end
                end
            end
        end
               
    end
    
    methods (Access=public)
        function [] = informClients(obj, propName)
            assert(isfield(obj.propInfo, propName));
            for i = 1:length(obj.propInfo.(propName).clients)
                client = obj.propInfo.(propName).clients{i};
                propNameClient = obj.propInfo.(propName).clientPropNames{i};
                
                client.(propNameClient) = obj.(propName);
            end
        end
    end
    
end

