classdef InvKinTracker < TrajectoryGenerators.TrajectoryTracker.TrajectoryTracker
    
    properties
        
        env;
        
    end
        
    
    methods
              
        function obj = InvKinTracker(dataManager, env)     
            numJoints = env.dimAction;
            obj = obj@TrajectoryGenerators.TrajectoryTracker.TrajectoryTracker(dataManager, numJoints);  
            
            obj.registerOptionalParameter('PGains', false, numJoints , zeros(1, numJoints ), 100 * ones(1, numJoints), 'parameters');
            obj.registerOptionalParameter('DGains', false, numJoints , zeros(1, numJoints ), 100 * ones(1, numJoints), 'parameters');
            obj.registerOptionalParameter('PDControllerFFGain', false, numJoints , zeros(1, numJoints ), 100 * ones(1, numJoints), 'parameters');
            
            obj.registerOptionalParameter('useInvDyn', false, 1 , false, true, 'parameters');
            
            obj.setIfNotEmpty('PGains', 100 * ones(1,obj.numJoints) )
            obj.setIfNotEmpty('DGains', 20 * ones(1,obj.numJoints) )
            obj.setIfNotEmpty('PDControllerFFGain', 1 * ones(1,obj.numJoints) )
            
            obj.setIfNotEmpty('useInvDyn', false );
            
            obj.env = env;            
                        
            obj.registerTrackingFunction();
            
            obj.addDataManipulationFunction('getTrackingControl', {'jointPositions', 'jointVelocities', obj.referenceTrajectory{:}, obj.additionalParameters{:}},...
                {'actions', 'jointRefPos', 'jointRefVel', 'dynUff'});
            obj.dataManager.addDataEntryForDepth(2,'jointRefPos', numJoints);
            obj.dataManager.addDataEntryForDepth(2,'jointRefVel', numJoints);
            obj.dataManager.addDataEntryForDepth(2,'dynUff', numJoints);
        end     
        
        function [action, jointRefPos, jointRefVel, uff] = getTrackingControl(obj, jointPositions, jointVelocities, referenceTrajectory, referenceTrajectoryD, referenceTrajectoryDD, varargin)
            obj.inputParameterDeMux(varargin);
            

            jointRefPos = jointPositions;
            
            for i = 1:size(jointPositions,1)
                for j = 1:30;%1e4
                    err = referenceTrajectory(i,:) - obj.env.getForwardKinematics(jointRefPos(i,:));
                    if ( norm(err) < 1e-2 )
                        break;
                    end
                    vel = 1e-2 * obj.env.getJacobian(jointRefPos(i,:))' * err';
                    jointRefPos(i,:) = jointRefPos(i,:) + vel';
                end
                if ( j == 1e4 )
                    keyboard
                end
            end
            
            jointRefVel = (jointRefPos - jointPositions) / obj.env.dt;            
            
            action = bsxfun(@times, (jointRefPos - jointPositions), obj.PGains) + bsxfun(@times, (jointRefVel - jointVelocities), obj.DGains);
            
            jointRefAcc = ( jointRefVel - jointVelocities) / obj.env.dt;
            uff = jointRefAcc;
            if ( obj.useInvDyn )
                for i = 1:size(jointPositions,1)
                    posvel = [jointPositions(i,:); jointVelocities(i,:) ];
                    [MMatrix, offest] = obj.env.getInvDyn(posvel(:));
                    uff(i,:) = offest + MMatrix * jointRefAcc(i,:)';
                    action(i,:) = action(i,:)' + uff(i,:)';
                    %                     action(i,:) = offest + MMatrix * jointRefAcc(i,:)';
                    %                     action(i,:) = action(i,:)' + offest ;
                end
            else
                action = action + bsxfun(@times, referenceTrajectoryDD, obj.PDControllerFFGain);
            end
        end
        
        
    end        
                  
end