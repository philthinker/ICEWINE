classdef QGMMZero < GMMOne
    %QGMMZero Unit quaternion Gaussian mixture model
    %   Used to model the unit quaternion trajectories only.
    %   Auxiliary unit quaternion is needed.
    %   Please use the log/exp map manually for efficiency.
    %   q = v + u : [qw qx qy qz]'
    %   
    %   Haopeng Hu
    %   2020.07.22
    %   All rights reserved
    %
    %   Notations:
    %   |   N:      num. of data
    %   |   M:      num. of demos
    %   |   D:      dim. of state
    %   |   Din:   dim. of query state
    %   |   Dout: dim. of queried state
    %   |   K:      num. of Gaussians
    
    properties
        q_a;        % 4 x 1, auxiliary unit quat
    end
    
    properties (Access = protected)
        bar_q_a;                % 4 x 1, conjugate of q_a
        quat_min = 1e-6;  % min quat distance
    end
    
    methods
        function obj = QGMMZero(K,D,q_a)
            %QGMMZero Init. the QGMM with K, D
            %   K: integer
            %   D: integer
            %   q_a: 4 x 1, auxiliary unit quat
            K = round(K);
            D = max(round(D),4);    % At least 4 dim
            obj = obj@GMMOne(K,D-1);
            obj.q_a = q_a;
            obj.bar_q_a = [q_a(1); -q_a(2); -q_a(3); -q_a(4)];
        end
        
        function eta = logmap(obj,q)
            %logmap Logrithmic map from S3 to R3
            %   log(q*bar_q_a)
            %   q: 4 x N, [qw,qx,qy,qz]' unit quat
            %   eta: 3 x N, [x,y,z]' point in tangent space centered at q_a
            tmpw = q(1,:) < 0;
            q( : , tmpw) = -q( : , tmpw);   % Double cover issue
            delta_q = obj.quatDistance(q);
            N = size(q,2);
            eta = zeros(3,N);
            v = delta_q(1,:);
            u = delta_q(2:4,:);
            for i = 1:N
                if norm(u(:,i)) < obj.quat_min
                    eta(:,i) = [0;0;0];
                else
                    eta(:,i) = acos(v(i))*u(:,i)/norm(u(:,i));
                end
            end
        end
        
        function q = expmap(obj,eta)
            %expmap Exponential map from R3 to S3
            %   exp(eta) = q*bar_q_a
            %   q = exp(eta)*q_a
            %   eta: 3 x N, point in tangent space centered at q_a
            %   q: 4 x N, unit quat
            N = size(eta,2);
            exp_eta = repmat([1,0,0,0]',[1,N]);
            for i = 1:N
                tmp_norm_eta = norm(eta(:,i));
                if tmp_norm_eta < obj.quat_min
                    exp_eta(:,i) = [1 0 0 0]';
                else
                    exp_eta(:,i) = [cos(tmp_norm_eta);  sin(tmp_norm_eta)*eta(:,i)/tmp_norm_eta];
                end
            end
            q = obj.quatProduct(exp_eta,repmat(obj.q_a,[1,N]));
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
            barQa = obj.bar_q_a;
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
            %quatProduct Quaternion product
            %   q1: 4 x N, quat
            %   q2: 4 x N, quat
            %   qOut: 4 x N, quat
            q1Matrix = obj.skewSymmetric(q1);
            qOut = q2;
            for i = 1:size(q2,2)
                qOut(:,i) = q1Matrix(:,:,i) * q2(:,i);
            end
        end
        % Figure
    end
    
    methods (Access = protected)
        % Functions related to unit quaternion
    end
end

