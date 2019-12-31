classdef PlanarForwardKinematics < Data.DataManipulator
    
    
    properties
        lengths
        numJoints
    end
    
    methods
        function obj = PlanarForwardKinematics(dataManager, dimensions)
            
            obj = obj@Data.DataManipulator(dataManager);                        
            
            % every joint has length 1
            obj.lengths 	= ones(1, dimensions);
            obj.numJoints = dimensions;
            
            obj.registerPlanarKinematicsFunctions();
        end
        
        function [] = registerPlanarKinematicsFunctions(obj)
            obj.addDataManipulationFunction('getForwardKinematics', {'jointPositions'}, {'endEffPositions'});
            obj.addDataManipulationFunction('getJacobian', {'jointPositions'}, {'J', 'endEffPositions'}, false);
        end
        
        function [] = registerEndEffectorVariables(obj)
            obj.dataManager.addDataEntry('steps.endEffPositions', 2, -ones(1,2) * sum(obj.lengths), ones(1,2) * sum(obj.lengths));
        end
        
        function [y] = getForwardKinematics(obj, theta, numLink)
            if (~exist('numLink', 'var'))
                numLink = obj.numJoints;
            end            

            y = zeros(size(theta, 1),2);
            for i = 1:numLink
                y = y + [sin(sum(theta(:,1:i), 2)), cos(sum(theta(:, 1:i), 2))] * obj.lengths(i);
            end            
        end
        
%         function [C] = getLinearizedForwardKinematics(obj, theta, numLink)
%             if (~exist('numLink', 'var'))
%                 numLink = obj.numJoints;
%             end
%             
%             C = zeros(2,obj.numJoints,size(theta, 1));
%             for i = 1:numLink
%                 C(:,1:i,:) = C(:,1:i,:) + repmat(permute([cos(sum(theta(:,1:i), 2)), -sin(sum(theta(:, 1:i), 2))] * obj.lengths(i),[2,3,1]),1,i,1);
%             end
%         end
        
        function [taskSpaceVelocity] = getTaskSpaceVelocity(obj, jointPosition, jointVelocities)
            taskSpaceVelocity = zeros(size(jointVelocities,1), 2);
            for i = 1:size(jointVelocities)
                [J, ~] = obj.getJacobian(jointPosition(i,:));
                taskSpaceVelocity(i,:) = (J * jointVelocities(i,:)')';
            end
        end        
                        
        function [J, si]  = getJacobian(obj, theta, numLink)
            % Get the jacobian at a given angle position q for a given link numLink
            if (~exist('numLink', 'var'))
                numLink = obj.numJoints;
            end
            
            si = obj.getForwardKinematics(theta, numLink);
            J  = zeros(2,obj.numJoints);
                        
            for j = 0:(numLink - 1)
                pj = [0, 0];
                for i = 1:j
                    pj = pj + [sin(sum(theta(1:i))), cos(sum(theta(1:i)))] * obj.lengths(i);
                end
                pj = -(si - pj);
                J([1 , 2], j + 1) = [-pj(2) pj(1)];
            end
        end
        
        function hnd = visualize(obj, theta, hnd, colorMap)
            
            if (size(theta,2) == obj.numJoints * 2)
                theta = theta(1:2:end);
            end
            
            mp = zeros(obj.numJoints + 1, 2);
            
            for i = 1:obj.numJoints
                mp(i + 1, :) = mp(i,:)  + [sin(sum(theta(1:i))), cos(sum(theta(1:i)))] * obj.lengths(i);
            end
            
            % display the link in red if the bounds are not good, blue if they are
            % good
            if (~exist('colorMap', 'var'))
                colorMap = repmat([0 0 1], obj.numJoints, 1);
            end
            
            if (size(colorMap,1) == 1)
                colorMap = repmat(colorMap, obj.numJoints, 1);
            end
            
            if (~exist('lw', 'var'))
                lw = 4.0;
            end
            
            if (~exist('fs', 'var'))
                fs = 26;
            end
            
            if (~exist('hnd', 'var'))
                hnd = figure;
            end
            
            figure(hnd)
            %clf;
            line([-sum(obj.lengths) sum(obj.lengths)], [0 0], 'LineStyle', '--');
            hold on; scatter(0,0,'.b','linewidth',10);
            
            for i = 1:obj.numJoints
                hold on; line([mp(i,1) mp(i+1,1)], [mp(i,2), mp(i+1,2)], 'linewidth',lw, 'color', colorMap(i,:));
                
                %r = 0.1;
                %rectangle('Position',[mp(i + 1)-r,mp1(2)-r,2*r,2*r],'Curvature',[1,1],...
                %'FaceColor','g')
                
            end
            
            xlabel('x-axis [m]', 'fontsize', fs);
            ylabel('y-axis [m]', 'fontsize', fs);
            axis([-sum(obj.lengths) sum(obj.lengths) -sum(obj.lengths) sum(obj.lengths)]);
        end
        
        function animate(obj, q, hnd, numSteps,colorMap)
            
            if (~exist('numSteps', 'var'))
                numSteps  = 1;
            end
            if (~exist('hnd', 'var'))
                hnd = figure;
            end
            if (~exist('colorMap', 'var'))
                colorMap = repmat(linspace(1, 0.1, size(q,1) / numSteps +1)', 1, 3);
            end
            
            for i = 1:numSteps:size(q,1)
                obj.visualize(q(i, :), hnd,colorMap(mod(i, size(colorMap,1)) + 1,:));
                pause(0.11)
            end
        end
    end
    
end
