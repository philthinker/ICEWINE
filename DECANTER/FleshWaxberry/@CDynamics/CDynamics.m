classdef CDynamics
    %CDynamics Cartesian dynamics for controller simulation
    %   Only [x y z ] + unit quaternion is supported, no externel force
    %   Impedance controller facilitated
    %   It only supports iterative usage
    
    properties
        dt;         % positive scalar, time step
        K;          % 6 x 6, Stiffness matrix
        D;          % 6 x 6, Damping matrix
    end
    
    methods
        function obj = CDynamics(dt)
            %CDynamics Init. the dynamics
            %   dt: positive scalar, time step
            obj.dt = max(0,dt);
            obj.K = eye(6);
            obj.D = eye(6);
        end
        
    end
    
    methods (Access = public)
        function [p,dp,ddp] = euclDynamics(obj,p,dp,hatp)
            %euclDynamics Euclidean dynamics equation
            %   ddp = K*(hatp - p) - D*dp
            %   p: 3 x 1, position
            %   dp: 3 x 1, velocity
            %   hatp: 3 x 1, reference position
            %   ddp: 3 x 1, acceleration
            euclK = obj.K(1:3,1:3);
            euclD = obj.D(1:3,1:3);
            ddp = euclK * (hatp - p) - euclD * dp;
            dp = dp + ddp * obj.dt;
            p = p + dp * obj.dt;
        end
        function [q,omega,domega] = quatDynamics(obj,q,omega,hatq)
            %quatDynamics Quaternion dynamics equation
            %   dq = 1/2 * omega * q
            %   domega = K*log(hatq * barq) - D*omega
            %   q(t + dt) = exp(dt/2 * omega) * q(t)
            %   q: 4 x 1, quat
            %   omega: 3 x 1, angular velocity
            %   hatq: 4 x 1, reference quat
            %   domega: 3 x 1, angular acceleration
            quatK = obj.K(4:6,4:6);
            quatD = obj.D(4:6,4:6);
            diffq = quatlog(quatmultiply(hatq',quatconj(q')))';
            domega = quatK * diffq(2:4) - quatD * omega;
            omega = omega + obj.dt * domega;
            q = quatmultiply( quatexp(obj.dt/2 * [0,omega']), q')';
        end
    end
end

