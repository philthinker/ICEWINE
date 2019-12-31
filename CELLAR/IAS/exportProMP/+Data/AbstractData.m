classdef AbstractData < Common.handleplus

    properties         
        dataManager;
    end
    
    methods
        function [obj] = AbstractData(dataManager)
            
            obj = obj@Common.handleplus();
            obj.dataManager = dataManager;
        end        
    end
    
    methods (Abstract)
        [numData] = getNumElements(obj, dataName)                       
        [dataMatrix] = getDataEntry(obj, elementName)
        [dataMatrix] = getDataEntryFlatIndex(obj, elementName, index)             
        [] = setDataEntry(obj, elementName, elementValue)
        [] = setDataEntryFlatIndex(obj, elementName, index, elementValue)                
    end
    
end