

function out = particleSwarmOptimizerSimple(costFun, nParticles, nIter, lowerBound, upperBound)
	kappa = 1;
	phi2 = 2.05;
	phi1 = 2.05;
	wDamp = .985;
	
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
	bestPos = zeros(nIter, nVar); % Array to Hold Best Posistion
	for iter = 1:nIter 
		for ii=1:nParticles
			particle(ii).vel = w*particle(ii).vel... % Update Velocity
				+ c1*rand(varSize).*(particle(ii).best.pos - particle(ii).pos)...
				+ c2*rand(varSize).*(globalBest.pos - particle(ii).pos);
			
			particle(ii).pos = particle(ii).pos + particle(ii).vel; % Update Posistion
			
			particle(ii).cost = costFun(particle(ii).pos); % Evaluation
			if particle(ii).cost < particle(ii).best.cost 
				particle(ii).best.pos = particle(ii).pos; % Update Personal Best Posistion
				particle(ii).best.cost = particle(ii).cost; % Update Personal Best Velocity
				if particle(ii).best.cost < globalBest.cost % Update Global Best
					globalBest = particle(ii).best;
				end
			end
		end
		bestCosts(iter) = globalBest.cost; % Saves Best Cost to Array
		bestPos(iter,:) = globalBest.pos; % Saves Position Cost to Array
		w = w * wDamp; % Sets w
	end
	bestCosts = bestCosts(1:iter-1); % Append Best Cost to Array
	bestPos = bestPos(1:iter-1,:); % Append Best Posistion to Array
	
	out.pop = particle;
	out.globalBest = globalBest;
	out.bestCost = bestCosts;
	out.bestPos = bestPos;
end
