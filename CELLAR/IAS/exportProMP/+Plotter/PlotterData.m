classdef PlotterData
    %PLOTTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods(Static)
        
        function [allEpisodes] = getDataEntryAllEpisodes(data, entryName, trajectoryIndex)
            depth = data.dataManager.getDataEntryDepth(entryName);
            
            if (~exist('trajectoryIndex', 'var'))
                trajectoryIndex = 1:data.getNumElements();
            end
            
            numElementsList = zeros(numel(trajectoryIndex, 1));
            for i = 1:numel(trajectoryIndex)
                numElementsList(i) = data.getNumElementsForIndex(depth, trajectoryIndex(i));
            end
            
            
            maxElements = max(numElementsList);
            
            allEpisodes = NaN(numel(trajectoryIndex), maxElements, data.getNumDimensions(entryName));
            
            for i = 1:numel(trajectoryIndex)
                episode = data.getDataEntry(entryName, trajectoryIndex(i));
                allEpisodes(i,1:size(episode,1),:) = episode;
            end
        end
        
        
        function [allEpisodes, figureHandles] = plotTrajectories(data, entryName, dimensions, figureHandles, trajectoryIndex)
            if (~exist('trajectoryIndex', 'var'))
                trajectoryIndex = 1:data.getNumElements();
            end
            allEpisodes = Plotter.PlotterData.getDataEntryAllEpisodes(data, entryName, trajectoryIndex);
            if (~exist('dimensions', 'var'))
                dimensions = 1:data.getNumDimensions(entryName);
            end
            for i = 1:length(dimensions)
                if (~exist('figureHandles', 'var') || length(figureHandles) < i)
                    figureHandles(i) = figure;
                else
                    figure(figureHandles(i));
                end
                hold on;
                plot(squeeze(allEpisodes(:,:, dimensions(i)))');
                title(sprintf('%s Dimension %d', entryName, dimensions(i)));
            end
        end
        
        function [meanValues, stdValues] = getMeanAndStdForEpisodes(data, entryName)
            numEpisodes = data.getNumElementsForDepth(1);
            data3D = Plotter.PlotterData.getDataEntryAllEpisodes(data, entryName, 1:numEpisodes);
            
            meanValues = permute(mean(data3D, 1), [2 3 1]);
            stdValues = permute(std(data3D, [], 1), [2 3 1]);
        end
        
        function [meanValues, stdValues, figureHandles] = plotTrajectoriesMeanAndStd(data, entryName, dimensions, figureHandles, lineProps)
            
            [meanValues, stdValues] = Plotter.PlotterData.getMeanAndStdForEpisodes(data, entryName);
            
            if(~exist('lineProps', 'var') ) lineProps = []; end
            if(~exist('figureHandles', 'var') ) figureHandles = []; end
            figureHandles = Plotter.PlotterData.plotMeanAndStd(meanValues, stdValues, entryName, dimensions, figureHandles, lineProps);
            
        end
        
        function figureHandles = plotMeanAndStd(meanValues, stdValues, entryName, dimensions, figureHandles, lineProps)
            
            if(~exist('lineProps', 'var') || isempty('lineProps') )
                lineProps = {'b', 'LineWidth', 2};
            end
            if (ischar(dimensions) && dimensions == ':')
                dimensions = 1:size(meanValues,2);
            end
            for i = 1:length(dimensions)
                if (~exist('figureHandles', 'var') || length(figureHandles) < i)
                    figureHandles(i) = figure;
                else
                    figure(figureHandles(i));
                end
                hold on;
                graphHnd = Plotter.shadedErrorBar(1:size(meanValues, 1), meanValues(:, dimensions(i)), 2 * stdValues(:, dimensions(i)),lineProps,0.5,false,false);
                title(sprintf('%s Dimension %d', entryName, dimensions(i)));
            end
            
        end
        
    end
    
end


