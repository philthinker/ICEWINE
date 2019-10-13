classdef IFbGainsProvider < Common.IASObject

    methods(Abstract) 
        %[Kp, Kd, kff, SigmaCtl] = getFeedbackGains(varargin)
        [ K, kff, SigmaCtl ] = getFeedbackGainsForT(obj,data,timesteps,state);
    end
    
end