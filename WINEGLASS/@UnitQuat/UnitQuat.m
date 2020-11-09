classdef UnitQuat
    %UnitQuat Unit quaternion series
    %   For easy use of unit quaternion  in LfD studies
    %   We always claim the format: [w x y z]'
    %
    %   Haopeng Hu
    %   2020.11.08
    %   All rights reserved
    %
    %   Notations:
    %   |   N: num. of data
    
    properties
        q;           % 4 x N, quats
        qa;         % 4 x 1, auxiliary quat
        eta;        % 3 x N, eta in tangent space centered at qa
    end
    
    properties (Access = protected)
        qMin;     % scalar, the minimum quat distance for computation
        bar_qa;   % 4 x 1, conjugate of qa
    end
    
    methods
        function obj = UnitQuat(Data,mod,flag)
            %UnitQuat Init. the object with unit quat data
            %   Data: 4 x N, N x 4, 3 x 3 x N, 4 x 4 x N, orientation data
            %   mod: integer, the data type
            %   |   0: 4 x N, q, quat data
            %   |   1: N x 4, q', quat data
            %   |   2: 3 x 3 x N, SO3 data
            %   |   3: 4 x 4 x N, SE3 data
            %   flag: boolean, true for quat. regulation (default:false)
            if nargin < 3
                flag = false;
            end
            if mod == 1
                obj.q = Data';
            elseif mod == 2
                obj.q = rotm2quat(Data)';
            elseif mod == 3
                obj.q = tform2quat(Data)';
            else
                obj.q = Data;
            end
            if flag
                % quat regulation
                obj.q = obj.quatRegulate(obj.q);
            end
            obj.qa = [1,0,0,0]';
            obj.bar_qa = obj.qa;
            obj.eta = [];
            obj.qMin = 1e-6;
        end
    end
    
    methods (Access = public)
        % Basic operations
        function obj = setQa(obj,qa)
            %setQa Set the auxiliary quaternion
            %   qa: 4 x 1, unit quat, [w x y z]'
            if nargin < 2
                qa = obj.q(:,end);
            end
            obj.qa = qa(1:4,1);
            obj.bar_qa = obj.qa;
            obj.bar_qa(2:4) = -obj.qa(2:4);
        end
        
        function quatOut = quatRegulate(obj,quatIn)
            %quatRegulate Regulate the quatIn to make w always > 0
            %   quatIn: 4 x N, quat data
            %   -----------------------------------------
            %   quatOut: 4 x N, regulated quat data
            N = size(quatIn,2);
            quatOut = [ones(1,N); zeros(3,N)];
            for i = 1:N
                tmpQuat = quatIn(:,i);
                tmpQuat = tmpQuat/norm(tmpQuat);
                if tmpQuat(1) < 0
                    tmpQuat = -tmpQuat;
                end
                quatOut(:,i) = tmpQuat;
            end
        end
        
        function etaOut = logMap(obj, qIn)
            %logMap The logrithmic map to the tangent space given the
            %auxiliary quat qa.
            %   qIn: 4 x N, quat data
            %   -----------------------------------------
            %   etaOut: 3 x N, eta data
            tmpw = qIn(1,:) < 0;
            qIn( : , tmpw) = -qIn( : , tmpw);   % Double cover issue
            delta_q = obj.quatDistance(qIn);
            N = size(qIn,2);
            etaOut = zeros(3,N);
            v = delta_q(1,:);
            u = delta_q(2:4,:);
            for i = 1:N
                if norm(u(:,i)) < obj.qMin
                    etaOut(:,i) = [0;0;0];
                else
                    etaOut(:,i) = acos(v(i))*u(:,i)/norm(u(:,i));
                end
            end
        end
        
        function qOut = expMap(obj, etaIn)
            %logMap The logrithmic map to the tangent space given the
            %auxiliary quat qa.
            %   qIn: 4 x N, quat data
            %   qa: 4 x 1, auxiliary quat (optional)
            %   -----------------------------------------
            %   etaOut: 3 x N, eta data
            N = size(etaIn,2);
            exp_eta = repmat([1,0,0,0]',[1,N]);
            for i = 1:N
                tmp_norm_eta = norm(etaIn(:,i));
                if tmp_norm_eta < obj.qMin
                    exp_eta(:,i) = [1 0 0 0]';
                else
                    exp_eta(:,i) = [cos(tmp_norm_eta);  sin(tmp_norm_eta)*etaIn(:,i)/tmp_norm_eta];
                end
            end
            qOut = obj.quatProduct(exp_eta,repmat(obj.qa,[1,N]));
        end
    end
    
    methods (Access = public)
        % Figure
    end
    
    methods (Access = public)
        % Data
        function [etaOut,obj] = computeEta(obj)
            %computeEta Compute the eta given q, qa
            %   eta: 3 x N, eta
            etaOut = obj.logMap(obj.q);
            obj.eta = etaOut;
        end
        function rotmOut = getROTM(obj)
            %getROTM Get the SO(3) data
            %   rotmOut: 3 x 3 x N, SO(3) data
            rotmOut = quat2rotm(obj.q');
        end
    end
    
    methods (Access = public)
        % Auxiliary functions
        function [delta_q] = quatDistance(obj,q)
            %quatDistance Quaternion distance to q_a, q*bar_q_a
            %   q: 4 x N, quat
            %   delta_q: 4 x N, quat
            %   q1 * q2 = v1v2 - u1'u2 + v1u2 + v2u1 + u1 x u2
            delta_q = q;
            barQa = obj.bar_qa;
            delta_q(1,:) = q(1,:).* barQa(1) - barQa(2:4)'*q(2:4,:);
            delta_q(2:4,:) = (q(1,:)' * barQa(2:4)')' + barQa(1)*q(2:4,:) + cross(q(2:4,:), repmat(barQa(2:4),[1,size(q,2)]),1 );
        end
        function [bar_q] = conjugateQuat(obj,q)
            %conjugateQuat Conjugate of quat q
            %   q: 4 x N, quat
            %   bar_q: 4 x N, quat
            bar_q = [q(1,:); -q(2,:); -q(3,:); -q(4,:)];
        end
        function [qMatrix] = skewSymmetric(obj,q)
            %skewSymmetric Generate the skew symmetric matrix for cross
            %porduct of quaternion
            %   q: 4 x N, quat
            %   qMatrix: 4 x 4 x N, skew symmetric matrics
            N = size(q,2);
            qMatrix = repmat(eye(4),[1,1,N]);
            for i = 1:N
                qMatrix(:,:,i) = [q(1,i),   -q(2,i),    -q(3,i),    -q(4,i);...
                                       q(2,i),     q(1,i),    -q(4,i),     q(3,i);...
                                       q(3,i),     q(4,i),      q(1,i),    -q(2,i);...
                                       q(4,i),    -q(3,i),      q(2,i),     q(1,i)];
            end
        end
        function [qOut] = quatProduct(obj,q1,q2)
            %quatProduct Quaternion product by skew-symmetric matrix
            %   q1: 4 x N, quat
            %   q2: 4 x N, quat
            %   qOut: 4 x N, quat
            q1Matrix = obj.skewSymmetric(q1);
            qOut = q2;
            for i = 1:size(q2,2)
                qOut(:,i) = q1Matrix(:,:,i) * q2(:,i);
            end
        end
    end
end

