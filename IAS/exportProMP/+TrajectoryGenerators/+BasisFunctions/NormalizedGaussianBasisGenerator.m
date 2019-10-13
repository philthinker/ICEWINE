classdef NormalizedGaussianBasisGenerator < TrajectoryGenerators.BasisFunctions.BasisGeneratorDerivatives
    properties
        muBasis
    end
    
    properties (SetObservable, AbortSet)
        widthFactorBasis = 1.0;
        numCentersOutsideRange = 2.0;
    end
    
    methods
        
        function obj = NormalizedGaussianBasisGenerator(dataManager, phaseGenerator, basisName)
            if (~exist('basisName', 'var'))
                basisName = 'basis';
            end
            
            obj = obj@TrajectoryGenerators.BasisFunctions.BasisGeneratorDerivatives(dataManager, phaseGenerator, basisName);
            
            obj.linkProperty('widthFactorBasis');
            obj.linkProperty('numCentersOutsideRange');
        end
        
        function sigma = getBasisWidth(obj)
            sigma = ones(1, obj.numBasis) / obj.numBasis * obj.widthFactorBasis;
        end
        
        
        function mu = getMu(obj)
            mu = linspace(-obj.numCentersOutsideRange / obj.numBasis * obj.widthFactorBasis, ...
                1 + obj.numCentersOutsideRange / obj.numBasis * obj.widthFactorBasis, obj.numBasis);
        end
        
        %         function [basis_n, basisD_n, basisDD_n] = generateBasis(obj, phase)
        %
        %             mu = obj.getMu();
        %
        %             sigma = obj.getBasisWidth();
        %             time_mu = bsxfun(@minus, phase, mu );
        %             at = bsxfun(@times, time_mu, 1./sigma);
        %
        %             basis = bsxfun(@times, exp( -0.5 * at.^2 ), 1./sigma/sqrt(2*pi) );
        %
        %             basis_sum = sum(basis,2);
        %
        %             basis_n = bsxfun(@times, basis, 1 ./ basis_sum);
        %
        %             % figure
        %             % plot(time,basis_n')
        %
        %
        %             time_mu_sigma = bsxfun(@times, -time_mu, 1./(sigma.^2) );
        %
        %             if (nargout > 1)
        %                 basisD =  time_mu_sigma .* basis;
        %                 basisD_sum = sum(basisD,2);
        %
        %                 basisD_n_a = bsxfun(@times, basisD, basis_sum);
        %                 basisD_n_b = bsxfun(@times, basis, basisD_sum);
        %                 basisD_n = bsxfun(@times, basisD_n_a - basisD_n_b, 1 ./(basis_sum.^2) );
        %
        %                 tmp =  bsxfun(@times,basis, -1./(sigma.^2) );
        %                 basisDD = tmp + time_mu_sigma .* basisD;
        %                 basisDD_sum = sum(basisDD,2);
        %
        %
        %                 basisDD_n_a = bsxfun(@times, basisDD, basis_sum.^2);
        %                 basisDD_n_b1 = bsxfun(@times, basisD, basis_sum);
        %                 basisDD_n_b = bsxfun(@times, basisDD_n_b1, basisD_sum);
        %
        %                 basisDD_n_c1 =  2 * basisD_sum.^2 - basis_sum .* basisDD_sum;
        %                 basisDD_n_c = bsxfun(@times, basis,  basisDD_n_c1);
        %
        %                 basisDD_n_d = basisDD_n_a - 2 .* basisDD_n_b + basisDD_n_c;
        %
        %                 basisDD_n = bsxfun(@times, basisDD_n_d, 1 ./ basis_sum.^3);
        %             end
        %         end
        
        function basis_n = generateBasis(obj, phase)
            
            mu = obj.getMu();
            
            sigma = obj.getBasisWidth();
            time_mu = bsxfun(@minus, phase, mu );
            at = bsxfun(@times, time_mu, 1./sigma);
            
            basis = bsxfun(@times, exp( -0.5 * at.^2 ), 1./sigma/sqrt(2*pi) );
            
            basis_sum = sum(basis,2);
            
            basis_n = bsxfun(@times, basis, 1 ./ basis_sum);
            
        end
        
        function basisD_n  = generateBasisD(obj, phase)
            
            mu = obj.getMu();
            
            sigma = obj.getBasisWidth();
            time_mu = bsxfun(@minus, phase, mu );
            at = bsxfun(@times, time_mu, 1./sigma);
            
            basis = bsxfun(@times, exp( -0.5 * at.^2 ), 1./sigma/sqrt(2*pi) );
            
            basis_sum = sum(basis,2);
            
            time_mu_sigma = bsxfun(@times, -time_mu, 1./(sigma.^2) );
            
            basisD =  time_mu_sigma .* basis;
            basisD_sum = sum(basisD,2);
            
            basisD_n_a = bsxfun(@times, basisD, basis_sum);
            basisD_n_b = bsxfun(@times, basis, basisD_sum);
            basisD_n = bsxfun(@times, basisD_n_a - basisD_n_b, 1 ./(basis_sum.^2) );
            
        end
        
        
        % Computing the second derivative
        function basisDD_n  = generateBasisDD(obj, phase)
            
            mu = obj.getMu();
            
            sigma = obj.getBasisWidth();
            time_mu = bsxfun(@minus, phase, mu );
            at = bsxfun(@times, time_mu, 1./sigma);
            
            basis = bsxfun(@times, exp( -0.5 * at.^2 ), 1./sigma/sqrt(2*pi) );
            
            basis_sum = sum(basis,2);
            
            time_mu_sigma = bsxfun(@times, -time_mu, 1./(sigma.^2) );
            
            basisD =  time_mu_sigma .* basis;
            basisD_sum = sum(basisD,2);
            
            tmp =  bsxfun(@times,basis, -1./(sigma.^2) );
            basisDD = tmp + time_mu_sigma .* basisD;
            basisDD_sum = sum(basisDD,2);
            
            basisDD_n_a = bsxfun(@times, basisDD, basis_sum.^2);
            basisDD_n_b1 = bsxfun(@times, basisD, basis_sum);
            basisDD_n_b = bsxfun(@times, basisDD_n_b1, basisD_sum);
            
            basisDD_n_c1 =  2 * basisD_sum.^2 - basis_sum .* basisDD_sum;
            basisDD_n_c = bsxfun(@times, basis,  basisDD_n_c1);
            
            basisDD_n_d = basisDD_n_a - 2 .* basisDD_n_b + basisDD_n_c;
            
            basisDD_n = bsxfun(@times, basisDD_n_d, 1 ./ basis_sum.^3);
            
        end
        
        function basisDDD_n  = generateBasisDDD(obj, phase)
            
%             mu = obj.getMu();
%             
%             sigma = obj.getBasisWidth();
%             time_mu = bsxfun(@minus, phase, mu );
%             at = bsxfun(@times, time_mu, 1./sigma);
%             
%             B = bsxfun(@times, exp( -0.5 * at.^2 ), 1./sigma/sqrt(2*pi) );            
%             SB = sum(B,2);
%             
%             G =  bsxfun(@times, B, bsxfun(@rdivide, time_mu, sigma.^2));
%             L =  bsxfun(@times, B, bsxfun(@rdivide, time_mu, sigma.^4));
%             F =  bsxfun(@times, B, bsxfun(@rdivide, time_mu.^2, sigma.^4));
%             H =  bsxfun(@times, B, bsxfun(@rdivide, time_mu.^3, sigma.^6));
%             
%             
%             q1 = -8 * bsxfun(@rdivide,H,SB) + 12 * bsxfun(@rdivide,L,SB);
%             
%             q2 = sum( -  bsxfun(@rdivide, B,sigma.^2) + 2 * F );
%             q2 = bsxfun(@rdivide, 12 * bsxfun(@times,G,q2),SB.^2);
%             
%             q3 = bsxfun(@rdivide,-12 *bsxfun(@times, B, sum(G)),sigma.^2);
%             q3 = bsxfun(@rdivide,q3,SB.^2);
%             
%             q4 = bsxfun(@rdivide,24* bsxfun(@times, F, sum(G)),SB.^2);
%             
%             q5 = - bsxfun(@rdivide, B, sigma.^2 ) + 2*F;
%             q5 = -24* bsxfun(@times,bsxfun(@times,B,sum(q5)),sum(G));
%             q5 = bsxfun(@rdivide,q5,SB.^3);
%             
%             q6 = - 48 * bsxfun(@rdivide,bsxfun(@times,G, sum(G).^2 ),SB.^3);
%             
%             q7 = 48 * bsxfun(@rdivide,bsxfun(@times,B, sum(G).^3 ),SB.^4);
%             
%             q8 = -bsxfun(@rdivide,bsxfun(@times,B,sum(-8*H+12*L)),SB.^2);
%             
%             
%             basisDDD_n = q1 + q2 + q3 + q4 + q5 + q6 + q7 + q8;
            
            
            step = 0.0000001;
            test1 = obj.generateBasisDD(phase+step);
            test2 = obj.generateBasisDD(phase-step);
            basisDDD_n = (test1-test2)./2./step;
            
        end
        
    end
end
