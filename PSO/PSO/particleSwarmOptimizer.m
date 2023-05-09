%%% particleSwarmOptimizer %%%
% costFun - Function to minimize
% nParticles - number of particles
% nIter - number of iteratoins
% lowerBound - array of lower bound for each variable [1 x nVar]
% upperBound - array of lower bound for each variable [1 x nVar]

%%% varargin: Name-Value Pairs %%%
% 'kappa', value
% 'phi1', value
% 'phi2', value
% 'wDamp', damp term
% 'minW', minimum value of w (inertia term)
% 'recordPart', value (numper of particles to record)
% 'minErr', value (minimum error to go to)
function out = particleSwarmOptimizer(costFun, nParticles, nIter, lowerBound, upperBound, varargin)
	defaultKappa = 1;
	defaultPhi1 = 2.05;
	defaultPhi2 = 2.05;
	defaultWDamp = .985;
	defaultMinW = .1;
	defaultRecordPart = 0;
	defaultMinErr = -Inf;

	lowerBound = lowerBound(:)';
	upperBound = upperBound(:)';
	
	p = inputParser;
	addParameter(p,'kappa',defaultKappa,@(x)isnumeric(x));
	addParameter(p,'phi1',defaultPhi1,@(x)isnumeric(x));
	addParameter(p,'phi2',defaultPhi2,@(x)isnumeric(x));
	addParameter(p,'wDamp',defaultWDamp,@(x)isnumeric(x));
	addParameter(p,'minW',defaultMinW,@(x)isnumeric(x));
	addParameter(p,'recordPart',defaultRecordPart,@(x)isnumeric(x) && isscalar(x) && (x >= 0));
	addParameter(p,'minError',defaultMinErr,@(x)isnumeric(x));
	
	parse(p,varargin{:});
	
	kappa = p.Results.kappa;
	phi1 = p.Results.phi1;
	phi2 = p.Results.phi2;
	wDamp = p.Results.wDamp;
	minW = p.Results.minW;
	recordPart = p.Results.recordPart;
	minErr = p.Results.minError;
	
	nVar = length(upperBound);
	varSize = [1, nVar];
	
	phi = phi1 + phi2;
	chi = 2*kappa/abs(2-phi-sqrt(phi^2-4*phi));
	w = chi;
	c1 = chi*phi1;
	c2 = chi*phi2;
	
	maxVel = 0.2*(upperBound - lowerBound);
	minVel = -maxVel;

	%%% Initialization
	emptyParticle.pos = [];
	emptyParticle.vel = [];
	emptyParticle.cost = [];
	emptyParticle.best.pos = [];
	emptyParticle.best.cost = [];
	
	particle = repmat(emptyParticle, nParticles, 1); % Create Population Array
	particleRec = repmat(emptyParticle, recordPart, nIter); % Create Population Array to be Recorded
	
	globalBest.cost = Inf; % Initialize Glabal Best
	for ii=1:nParticles % initalize population members
		particle(ii).pos = unifrand(lowerBound, upperBound, nVar); % Generate Random Solution
		particle(ii).vel = zeros(varSize); % Initialize Velocity
		particle(ii).cost = costFun(particle(ii).pos); % Evaluation
		
		particle(ii).best.pos = particle(ii).pos; % Update Personal Best Posistion
		particle(ii).best.cost = particle(ii).cost; % Update Personal Best Velocity
		
		if particle(ii).best.cost < globalBest.cost % Update Global Best
			globalBest.pos = particle(ii).pos;
			globalBest.cost = particle(ii).cost;
		end
	end
	
	bestCosts = zeros(nIter, 1); % Array to Hold Best Cost
	bestPos = zeros(nIter, nVar);
	iter = 1;
	while iter <= nIter && globalBest.cost > minErr
		for ii=1:nParticles
			particle(ii).vel = w*particle(ii).vel... % Update Velocity
				+ c1*rand(varSize).*(particle(ii).best.pos - particle(ii).pos)...
				+ c2*rand(varSize).*(globalBest.pos - particle(ii).pos);
			
			particle(ii).pos = particle(ii).pos + particle(ii).vel; % Update Posistion
			particle(ii).vel = max(particle(ii).vel, minVel); % Apply Velocity Lower Bound
			particle(ii).vel = min(particle(ii).vel, maxVel); % Apply Velocity Upper Bound
			particle(ii).pos = max(particle(ii).pos, lowerBound); % Apply Position Lower Bound
			particle(ii).pos = min(particle(ii).pos, upperBound); % Apply Position Upper Bound
			
			particle(ii).cost = costFun(particle(ii).pos); % Evaluation
			if particle(ii).cost < particle(ii).best.cost 
				particle(ii).best.pos = particle(ii).pos; % Update Personal Best Posistion
				particle(ii).best.cost = particle(ii).cost; % Update Personal Best Velocity
				if particle(ii).best.cost < globalBest.cost % Update Global Best
					globalBest = particle(ii).best;
				end
			end
			if ii <= recordPart
				particleRec(ii,iter) = particle(ii);
			end
		end
		bestCosts(iter) = globalBest.cost; % Saves Best Cost to Array
		bestPos(iter,:) = globalBest.pos; % Saves Position Cost to Array
		w = max(w * wDamp, minW); % Sets w
		iter = iter + 1;
	end
	bestPos = bestPos(1:iter-1,:);
	bestCosts = bestCosts(1:iter-1);
	particleRec = particleRec(:,1:iter-1);
	
	out.pop = particle;
	out.popRec = particleRec;
	out.globalBest = globalBest;
	out.bestCost = bestCosts;
	out.bestPos = bestPos;
end
