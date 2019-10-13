classdef NormalizedBSplines < TrajectoryGenerators.BasisFunctions.BasisGeneratorDerivatives
    
    properties (SetObservable, AbortSet)
        bSplineDeg = 4;
        bSplineOffsetCoeff = 8;
        normilizeBasis = true;
    end
    
    methods
        
        function obj = NormalizedBSplines(dataManager, phaseGenerator, basisName)
            if (~exist('basisName', 'var'))
                basisName = 'basis';
            end
            
            obj = obj@TrajectoryGenerators.BasisFunctions.BasisGeneratorDerivatives(dataManager, phaseGenerator, basisName);
            
            obj.linkProperty('bSplineDeg');
            obj.linkProperty('bSplineOffsetCoeff');
            obj.linkProperty('normilizeBasis');
            
        end
        
        function B = getB(obj,i,intv,deg,t)
            
            if ( deg == 0 )
                B = and(intv(i)<=t , t<intv(i+1));
                return
            end
            
            B = (t-intv(i))/(intv(i+deg)-intv(i)).*obj.getB(i,intv,deg-1,t) + ...
                (intv(i+deg+1)-t)/(intv(i+deg+1)-intv(i+1)).*obj.getB(i+1,intv,deg-1,t);
            
        end
        
        function BD = getBD(obj,i,intv,deg,t)
            
            if ( deg == 0 )
                BD = zeros(size(t));
                return
            end
            
            BD = 1/(intv(i+deg)-intv(i)).*obj.getB(i,intv,deg-1,t) + ...
                (t-intv(i))/(intv(i+deg)-intv(i)).*obj.getBD(i,intv,deg-1,t) + ...
                -1/(intv(i+deg+1)-intv(i+1)).*obj.getB(i+1,intv,deg-1,t) +...
                (intv(i+deg+1)-t)/(intv(i+deg+1)-intv(i+1)).*obj.getBD(i+1,intv,deg-1,t);
            
        end
        
        function BD = getBDD(obj,i,intv,deg,t)
            
            if ( deg == 0 )
                BD = zeros(size(t));
                return
            end
            
            BD = 1/(intv(i+deg)-intv(i)).*obj.getBD(i,intv,deg-1,t) + ...
                1/(intv(i+deg)-intv(i)).*obj.getBD(i,intv,deg-1,t) + ...
                (t-intv(i))/(intv(i+deg)-intv(i)).*obj.getBDD(i,intv,deg-1,t) + ...
                -1/(intv(i+deg+1)-intv(i+1)).*obj.getBD(i+1,intv,deg-1,t) +...
                -1/(intv(i+deg+1)-intv(i+1)).*obj.getBD(i+1,intv,deg-1,t) + ...
                (intv(i+deg+1)-t)/(intv(i+deg+1)-intv(i+1)).*obj.getBDD(i+1,intv,deg-1,t);
            
        end
        
        function basis_n = generateBasis(obj, phase)
            
            nSplines = obj.numBasis;
            deg = obj.bSplineDeg;
            
            offset = obj.bSplineOffsetCoeff/(deg+nSplines+1);
            intv = linspace(0-offset,1+offset,deg+nSplines+1);
            
            basis = zeros(length(phase),obj.numBasis);
            for i=1:nSplines
                basis(:,i) = obj.getB(i,intv,deg,phase);
            end
            
            if ( obj.normilizeBasis )
                basis_sum = sum(basis,2);
                basis_n = bsxfun(@times, basis, 1 ./ basis_sum);
            else
                basis_n = basis;
            end
            
        end
        
        function basisD_n  = generateBasisD(obj, phase)
            
            nSplines = obj.numBasis;
            deg = obj.bSplineDeg;
            
            offset = obj.bSplineOffsetCoeff/(deg+nSplines+1);
            intv = linspace(0-offset,1+offset,deg+nSplines+1);
            
            basis = zeros(length(phase),obj.numBasis);
            for i=1:nSplines
                basis(:,i) = obj.getB(i,intv,deg,phase);
            end
            
            basisD = zeros(length(phase),obj.numBasis);
            for i=1:nSplines
                basisD(:,i) = obj.getBD(i,intv,deg,phase);
            end
            
            if ( obj.normilizeBasis )
                basisD_n = bsxfun(@times,basisD, 1./sum(basis,2))...
                    - bsxfun(@times,basis,  sum(basisD,2) ./ sum(basis,2).^2);
            else
                basisD_n = basisD;
            end
            
            %             tic
            %             basisD_n_e = zeros(length(phase),obj.numBasis);
            %             for i=1:nSplines
            %                 basisD_n_e(:,i) = basisD(:,i) ./ sum(basis,2) ...
            %                     - basis(:,i) ./ sum(basis,2).^2 .* sum(basisD,2);
            %             end
            %             toc
            %             tic
            %             basisD_n = bsxfun(@times,basisD, 1./sum(basis,2))...
            %                 - bsxfun(@times,basis,  sum(basisD,2) ./ sum(basis,2).^2);
            %             toc
            
            %             sum(sum(abs(basisD_n-basisD_n_e)))
            
        end
        
        function basisDD_n  = generateBasisDD(obj, phase)
            nSplines = obj.numBasis;
            deg = obj.bSplineDeg;
            
            offset = obj.bSplineOffsetCoeff/(deg+nSplines+1);
            intv = linspace(0-offset,1+offset,deg+nSplines+1);
            
            basis = zeros(length(phase),obj.numBasis);
            for i=1:nSplines
                basis(:,i) = obj.getB(i,intv,deg,phase);
            end
            
            basisD = zeros(length(phase),obj.numBasis);
            for i=1:nSplines
                basisD(:,i) = obj.getBD(i,intv,deg,phase);
            end
            
            basisDD = zeros(length(phase),obj.numBasis);
            for i=1:nSplines
                basisDD(:,i) = obj.getBDD(i,intv,deg,phase);
            end
            
            if ( obj.normilizeBasis )
                basisDD_n = bsxfun(@times, basisDD, 1./sum(basis,2)) ...
                    - bsxfun(@times,basisD, 2*sum(basisD,2) ./ sum(basis,2).^2 )...
                    - bsxfun(@times,basis, sum(basisDD,2) ./ sum(basis,2).^2 )...
                    +bsxfun(@times,basis, sum(basisD,2) ./ sum(basis,2).^3 .* sum(basisD,2) );
            else
                basisDD_n = basisDD;
            end
            
            %             tic
            %             basisDD_n_e = zeros(length(phase),obj.numBasis);
            %             for i=1:nSplines
            %                 basisDD_n_e(:,i) = basisDD(:,i) ./ sum(basis,2) ...
            %                     - basisD(:,i) ./ sum(basis,2).^2 .* sum(basisD,2)...
            %                     - basisD(:,i) ./ sum(basis,2).^2 .* sum(basisD,2) ...
            %                     - basis(:,i) .* sum(basisDD,2) ./ sum(basis,2).^2 ...
            %                     + basis(:,i) .* sum(basisD,2) ./ sum(basis,2).^3 .* sum(basisD,2);
            %             end
            %             toc
            
            %             tic
            %             basisDD_n = bsxfun(@times, basisDD, 1./sum(basis,2)) ...
            %                 - bsxfun(@times,basisD, 2*sum(basisD,2) ./ sum(basis,2).^2 )...
            %                 - bsxfun(@times,basis, sum(basisDD,2) ./ sum(basis,2).^2 )...
            %                 +bsxfun(@times,basis, sum(basisD,2) ./ sum(basis,2).^3 .* sum(basisD,2) );
            %             toc
            
            %             sum(sum(abs(basisDD_n-basisDD_n_e)))
            
        end
        
        %% Numerical derivatives
        %         function basisD_n  = generateBasisD(obj, phase)
        %
        %             phaseD  = diff(phase);
        %             phaseDD = diff(phase,2);
        %
        %             % basis_n = obj.generateBasis([phase;phase(end)+phaseD(end)]);
        %             basis_n = obj.generateBasis([phase(1);phase;]);
        %             basisD_n = bsxfun(@times,diff(basis_n),1./[phaseD;phaseD(end)+phaseDD(end)]);
        %
        %         end
        
        
        % Computing the second derivative
        %         function basisDD_n  = generateBasisDD(obj, phase)
        %
        %             phaseD   = diff(phase);
        %             phaseDD  = diff(phase,2);
        %             phaseDDD = diff(phase,3);
        %
        %             basis_n = obj.generateBasis([phase;phase(end)+phaseD(end);phase(end)+2*phaseD(end)]);
        %             basisDD_n = bsxfun(@times,diff(basis_n,2),1./([phaseDD;phaseDD(end)+phaseDDD(end)]).^2);
        %         end
        
    end
end
