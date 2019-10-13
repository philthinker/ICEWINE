classdef DataPreprocessor < Common.IASObject
    
   properties
        iteration = 0;
        
        dataNamePreprocessor = 'data';
   end
   
   % Class methods
   methods
      function obj = DataPreprocessor(varargin)
            obj = obj@Common.IASObject(varargin{:});
      end 
            
      function [data] = preprocessData(obj, data)
          
      end
      
      function [] = preprocessDataCollection(obj, dataCollection)
          dataNew = obj.preprocessData(dataCollection.getStandardData());
          dataCollection.addDataObject(dataNew, obj.dataNamePreprocessor);          
      end
   
      function [] = setIteration(obj, iteration)
          obj.iteration = iteration;
      end
   end
end
