classdef SettingsManager < handle;
    
    properties(SetAccess = protected)
        collection
        defaultIds = {'default'}
        debugMode = false;
        nextClientId = 1;
    end
    
    methods(Access=private)
        
        function obj = SettingsManager()
            persistent instance;
            
            if(isempty(instance))
                instance = obj;
            else
                obj = instance;
            end
        end
        
    end
    
    methods(Static)

        function activateDebugMode()
            obj = Common.SettingsManager();
            obj.debugMode = true;
        end
        
        function deactivateDebugMode()
            obj = Common.SettingsManager();
            obj.debugMode = false;
        end
        
        function tf = inDebugMode()
            obj = Common.SettingsManager();
            tf = obj.debugMode;
        end
        
        function pushDefaultId(id)
            obj = Common.SettingsManager();
            obj.defaultIds{end+1} = id;
        end
        
        function id = getRootId()
            obj = Common.SettingsManager();
            id = obj.defaultIds{1};
        end
        
        function id = getDefaultId()
            obj = Common.SettingsManager();
            id = obj.defaultIds{end};
        end
        
        function id = popDefaultId()
            obj = Common.SettingsManager();
            id = obj.defaultIds{end};
            if(numel(obj.defaultIds) > 1)
                obj.defaultIds(end) = [];
            end
        end
        
        function set = getSettings(id)
            obj = Common.SettingsManager();
            if(isfield(obj.collection, id))
                set = obj.collection.(id);
            else
                set = [];
            end
        end
        
        function setSettings(settings)
            obj = Common.SettingsManager();
            obj.collection.(settings.id) = settings;
        end
        
        function setRootSettings(settings)
            obj = Common.SettingsManager();
            obj.collection.(obj.defaultIds{end}) = settings;
        end        
        
        function delSettings(id)
            obj = Common.SettingsManager();
            if(isfield(obj.collection, id))
                obj.collection = rmfield(obj.collection, id);
            end
        end
        
        function nextClientId = getNextClientId()
            obj = Common.SettingsManager();
            nextClientId = sprintf('client_%03d',obj.nextClientId);
            obj.nextClientId = obj.nextClientId+1;
        end
        
    end
    
end
