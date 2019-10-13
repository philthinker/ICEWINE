classdef TransitionFunction < Common.IASObject & Data.DataManipulator
    
    properties (GetAccess=public, SetAccess=private)
        isPeriodicState
        minRangeState
        maxRangeState
        minRangeAction
        maxRangeAction
        
        dimState
        dimAction
        
        %useTransitionProbabilities = false;
    end
    
    methods
        function obj = TransitionFunction(rootSampler, dimState, dimAction)
            
           obj = obj@Common.IASObject();
           obj = obj@Data.DataManipulator(rootSampler.getDataManagerForSampler());
           
           obj.dataManager.addDataEntry('steps.states', dimState);
           obj.dataManager.addDataEntry('steps.nextStates', dimState);
           obj.dataManager.addDataEntry('steps.actions', dimAction);
           
           obj.dataManager.setRestrictToRange('actions', true);
           
           obj.addDataManipulationFunction('transitionFunction', {'states', 'actions'}, {'nextStates'});
           obj.addDataFunctionAlias('sampleNextState', 'transitionFunction'); 
           
           obj.addDataManipulationFunction('initStateFromContexts', {'contexts'}, {'states'});
           obj.addDataFunctionAlias('sampleInitState', 'initStateFromContexts');                       
           
        end              
        
        function [initStates] = initStateFromContexts(obj, contexts)
            initStates = contexts(:, 1:obj.dimState);            
        end
           
%         function [] = enableTransitionProbabilities(obj, enableProp)
%             obj.useTransitionProbabilities = enableProp;
%             if (enableProp)
%                 obj.dataManager.addDataEntry('steps.logQSsa', 1);
%                 obj.setTransitionOutput('nextStates', 'logQSsa');                   
%             else
%                 obj.setTransitionOutput('nextStates');
%             end                          
%         end
        
        function [stateDiff] = getStateDifference(obj, state1, state2)
            stateDiff = (state1 - state2);
            
            period = obj.maxRangeState - obj.minRangeState;
            period = repmat(period, size(state1,1), 1);
            
            indexUB = bsxfun(@and, bsxfun(@gt, state, maxState), obj.isPeriodicState);
            stateDiff(indexUB) = stateDiff(indexUB) - period(indexUB);
            
            indexLB = bsxfun(@and, bsxfun(@lt, state, minState), obj.isPeriodicState);
            stateDiff(indexLB) = stateDiff(indexLB) + period(indexLB);
        end
        
        function [state] = projectStateInPeriod(obj, state)
            period = obj.maxRangeState - obj.minRangeState;
            period = repmat(period, size(state,1), 1);
            minState = obj.minRangeState;
            maxState = obj.maxRangeState;
            
            indexUB = bsxfun(@and, bsxfun(@gt, state, maxState), obj.isPeriodicState);           
            state(indexUB) = state(indexUB) - period(indexUB);
            
            indexLB = bsxfun(@and, bsxfun(@lt, state, minState), obj.isPeriodicState);   
            state(indexLB) = state(indexLB) + period(indexLB);
        end
        
        function [] = initObject(obj)
            obj.isPeriodicState = obj.dataManager.getPeriodicity('states');
            obj.dimState = obj.dataManager.getNumDimensions('states');
            obj.dimAction = obj.dataManager.getNumDimensions('actions');   
            obj.minRangeState = obj.dataManager.getMinRange('states');
            obj.maxRangeState = obj.dataManager.getMaxRange('states');
            
            obj.minRangeAction = obj.dataManager.getMinRange('actions');
            obj.maxRangeAction = obj.dataManager.getMaxRange('actions');            
        end
        
        function [] = setTransitionInputs(obj, varargin)
            obj.setInputArguments('transitionFunction', varargin);
        end
        
        function [] = setTransitionOutput(obj, varargin)
            obj.setOutputArguments('transitionFunction', varargin);
        end                          
       
    end
    
    methods (Abstract)
        [vargout] = transitionFunction(obj, varargin)        
        
    end
end