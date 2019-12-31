function [Sigma, cholSigma] = regularizeCovariance(Sigma, priorCov, numEffectiveSamples, priorCovWeight)


    count = 1;
    %Sigma_temp =  Sigma;
    
    while (count < 100)
        
                Sigma_temp = (Sigma * numEffectiveSamples + priorCov * priorCovWeight) / (numEffectiveSamples + priorCovWeight);
%         Sigma_temp = bsxfun(@plus,Sigma * numEffectiveSamples , diag(priorCov) * priorCovWeight) / (numEffectiveSamples + priorCovWeight);
        
        priorCovWeight = priorCovWeight * 2;
        count = count + 1;
        
        try 
            cholSigma = chol(Sigma_temp);
            if(all(eig(Sigma_temp)>0))
                Sigma = Sigma_temp;
                return
            end
            
        catch E
           
        end
        
    end
    disp(Sigma); disp(eig(Sigma))
    error('Could not find decomposition for covariance matrix... HELP!!!\n');


end