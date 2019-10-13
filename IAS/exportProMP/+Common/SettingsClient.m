classdef SettingsClient < handle;
    
    % Base class for all IASObjects. Implements basic functionality for
    % registering properties of an object in the global parameter pool.
    properties(SetAccess=private)
        settingsClientId = [];     
        localPropertyList = {};
    end
    
    methods
        
        function obj = SettingsClient()
            obj.settingsClientId = Common.SettingsManager.getNextClientId();
        end
        
        function linkProperty(obj,clientPropName,settingsPropName,settings)
            % Registers the property 'clientPropName' of the object into
            % the parameter pool.
            % The parameter 'settingsPropName' sets the global name of the
            % property used in the parameter pool (default is the same name
            % as the internal name used for the class). The parameter
            % settings specifies the parameter pool where we want to
            % register the property. Default is the global parameter pool.
            % If a property is linked and the property alreay exists in the 
            % parameter pool, the value from the parameter pool is used to 
            % overwrite the value of the property of the object. If the 
            % property does not exist in the parameter pool, it property is 
            % registered in the parameter pool. The value in the parameter 
            % pool is in this case set to the current value of the local 
            % property.
            % NOTE: ALL properties that are linked must be declared as
            % 'AbortSet' and 'SetObservable'!
            if(~exist('settingsPropName','var')||isempty(settingsPropName))
                settingsPropName = clientPropName;
            end
            
            if(~exist('settings','var')||isempty(settings))
                settings = Common.Settings();
            end
            
%             if(isempty(obj.settingsClientId))
%                 if(Common.SettingsManager.inDebugMode())
%                     warning('PolicySearchToolbox:Common:SettingsClient:Debug:noClientId','SettingsClient should get his clientId in the constructor. Make sure the constructor has been called.');
%                 end
%                 obj.settingsClientId = Common.SettingsManager.getNextClientId();
%             end
            
            settings.linkProperty(obj,clientPropName,settingsPropName);
            obj.localPropertyList{end + 1} = settingsPropName;
                       
        end
        
        function printProperties(obj)
            
            for i = 1:length(obj.localPropertyList)
                value = Common.Settings().getProperty(obj.localPropertyList{i});
                if (isa(value, 'double'))
                    fprintf('%s: %f\n', obj.localPropertyList{i}, value); 
                end
                
                if (isa(value, 'char'))
                    fprintf('%s: %s\n', obj.localPropertyList{i}, value); 
                end
                
            end
        end
        
        function unlinkProperty(obj,clientPropName, settings)
           warning('unlinking property not used any more. Statement can be deleted.');
        end
        
%         function settingsPropertyChanged(obj, settings, settingsPropName, clientPropName)
%             settingsPropValue = settings.getProperty(settingsPropName);
%             
%             %fprintf('settingsPropertyChanged %s.%s(%d) -> %s.%s(%d)\n',obj.settingsClientId,clientPropName,obj.(clientPropName),settings.id,settingsPropName,settingsPropValue);
%             if( ~isequal(obj.(clientPropName),settingsPropValue))
%                 %fprintf('\tDONE\n');
%                 obj.(clientPropName) = settingsPropValue;
%             end
%         end
%         
    end
    
end

