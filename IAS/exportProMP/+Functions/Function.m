classdef Function < Functions.MappingInterface
    % The Function class is the base class for every function. 
%
% It registers the data manipulation function <tt>getExpectation</tt> and
% defines it as an abstract function to make sure every function implements that function.
    properties (SetAccess = protected)

    end
    
    methods
        function obj = Function()
        end

        function [] = registerMappingInterfaceFunction(obj)
            if (obj.registerDataFunctions)
                obj.addMappingFunction('getExpectation');
            end
        end

% Geri: Depricated - use Plotter.PlotterFunctions        
%         function [standardInput, outputValue] = plotOutputFunctionSlice1D(obj, dim1, minValue, maxValue, standardInput, numBins, varargin)
%             
%             stateValues1 = linspace(minValue, maxValue, numBins);
%             standardInput = repmat(standardInput, numBins, 1);
%             standardInput(:,dim1) = stateValues1;
%             
%             outputValue = obj.getExpectation(numBins, standardInput);
%             plot(standardInput, outputValue);
%         end
%         
%         function [] = plotOutputFunctionSlice2D(obj, dim1, dim2, minValue, maxValue, standardInput, numBins)
%             stateValues1 = linspace(minValue(1), maxValue(1), numBins);
%             stateValues2 = linspace(minValue(2), maxValue(2), numBins);
%             
%             [stateValuesGrid1, stateValuesGrid2] = meshgrid(stateValues1, stateValues2);
%             
%             stateValuesGrid1 = stateValuesGrid1(:);
%             stateValuesGrid2 = stateValuesGrid2(:);
%             
%             standardInput = repmat(standardInput, size(stateValuesGrid1,1), 1);
%             standardInput(:,dim1) = stateValuesGrid1;
%             standardInput(:,dim2) = stateValuesGrid2;            
%             
%             outputValue = obj.getExpectation(numBins, standardInput);
%             surf(stateValues1, stateValues2, outputValue);
%         end
    end
    
    methods (Abstract)
        % Returns the expectation of the Function.
        [value] = getExpectation(obj, numElements, varargin)
    end
    
end
