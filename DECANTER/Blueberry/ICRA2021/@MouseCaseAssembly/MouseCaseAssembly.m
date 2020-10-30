classdef MouseCaseAssembly
    %MouseCaseAssembly Mouse case assembly with Panda and Parallel robots
    %   Good luck!
    %   
    %   Haopeng Hu
    %   2020.10.30
    %   All rights reserved
    %   
    %   Notations
    %   |   N:  num. of data
    %   |   M:  num. of demonstration
    
    properties
        label;      % String, the experiment label
        pPolicy;    % GMM policy for position
        qPolicy;    % QGMM policy for orientation
    end
    
    methods
        function obj = MouseCaseAssembly(label)
            %MouseCaseAssembly Good luck!
            %   label: String, the experiment label
            obj.label = label;
            obj.pPolicy = [];
            obj.qPolicy = [];
        end
    end
    
    methods (Access = public)
        % Data pre-process
        function [frankaData] = frankaDataFormulate(obj,frankaData)
            %frankaDataFormulate Formuate the frankaData var. for usage
            %   frankaData: 1 x M struct array
            %   |   OTEE: N x 16, end-effector data in Cartesian space
            M = size(frankaData,2);
            for i = 1:M
                % SE(3) data
                frankaData(i).rawSE3 = fold2SE3(frankaData(i).OTEE);
                % SE(3) data w.r.t. the end pose frame - 4 x 4 x N
                % Position data w.r.t. the end pose frame - 3 x N
                % Unit quaternion w.r.t. the end pose frame - 4 x N
                frankaData(i).SE3 = frankaData(i).rawSE3;
                N = size(frankaData(i).SE3,3);
                frankaData(i).p = zeros(3,N);
                frankaData(i).q = zeros(4,N);
                for j = 1:N
                    frankaData(i).SE3(:,:,j) = frankaData(i).rawSE3(:,:,end)\frankaData(i).SE3(:,:,j);
                    frankaData(i).p(:,j) = permute( frankaData(i).SE3(1:3,4,j), [1,2,3] );
                    frankaData(i).q(:,j) = tform2quat(frankaData(i).SE3(:,:,j))';
                end
                % Auxiliary variables
                % time: 1 x N, time variable
                % A: 4 x 4 x 2, task frames - orientation
                % b: 4 x 2, task frames - position
                % qa: 4 x 1, auxiliary quaternion
                % phase: 1 x N, reserved
                frankaData(i).time = linspace(0,1,N);
                frankaData(i).A = repmat(eye(4),[1,1,2]);
                frankaData(i).b = zeros(4,2);
                frankaData(i).A(2:4,2:4,1) = frankaData(i).rawSE3(1:3,1:3,1);
                frankaData(i).A(2:4,2:4,2) = frankaData(i).rawSE3(1:3,1:3,end);
                frankaData(i).b(2:4,1) = frankaData(i).rawSE3(1:3,4,1);
                frankaData(i).b(2:4,2) = frankaData(i).rawSE3(1:3,4,end);
                frankaData(i).qa = tform2quat(frankaData(i).rawSE3(:,:,end));
                frankaData(i).phase = frankaData(i).time;
            end
        end
    end
end

