classdef BallTracker < handle
    properties
        trajectory;     % Trajectory struct
        config;         % Configuration parameters
		cameras;        % Camera objects
		UKF;
    end

    methods
        % Constructor
        function obj = BallTracker(x0, configName)
            obj.config = obj.getConfigStruct(configName);  % Load configuration
            obj.trajectory.x0 = x0;                        % Set initial state
        end

		function setCameras(obj)
			cams = obj.config.Cameras;
			numCams = length(cams);
			obj.cameras = Camera.empty(0, numCams);
			for ii=1:numCams
				cam = cams(ii);
				obj.cameras(ii) = Camera([cam.HFOV, cam.VFOV], cam.depth);
				obj.cameras(ii).setTranslation(cam.pos);
				obj.cameras(ii).setRotation(cam.az, cam.el, cam.roll);
				obj.cameras(ii).name = cam.id;
			end
		end

        % Run the simulation
        function runSim(obj)
			obj.simTrajectory()
			obj.getCamObs()
			obj.runUKF()
		end

		function simTrajectory(obj)
			dt = obj.config.SimSettings.dt;
            tspan = 0:dt:obj.config.SimSettings.maxTime;  % Time span for the simulation

            % Set options for ODE solver, including event function
            options = odeset('Events', @obj.groundEvent);
            
            % Solve the ODE
            [time, xTrue] = ode45(@(t, x) obj.motionOfBall(x), tspan, obj.trajectory.x0, options);
			obj.trajectory.time = time;
			obj.trajectory.xTrue = xTrue;
			obj.trajectory.numSteps = length(time);
		end

		function getCamObs(obj)
			numCams = length(obj.cameras);
			for ii=1:numCams
				cam = obj.cameras(ii);
				cam.global2UVA(obj.trajectory.xTrue(:,1:3), obj.trajectory.time, obj.config.GolfBall.radius)
				cam.addCamNoise()
			end
		end

		function runUKF(obj)

			N = 9;  % number of state variables
			alpha = 0.05;  % spread of sigma points
			kappa = 0;  % secondary scaling factor
			beta = 2;  % optimal for gaussian distribution

			x0 = [0 -150 0 0 0 0 0 0 0].';

			posP = [100, 100, 100];
			velP = [100, 100, 100];
			omgP = [100, 100, 100];
			P0 = diag([posP, velP, omgP].^2);
				
			lambda = alpha^2 * (N + kappa) - N;
			w0Mean = lambda / (N + lambda);  % weight for the first sigma point for mean
			w0Cov = w0Mean + (1 - alpha^2 + beta);  % weight for the first sigma point for covariance
			wi = 1 / (2*(N + lambda));  % is a weight for the other sigma points when computing mean or covariance

			wCov = diag([w0Cov, repmat(wi, 1, 2*N)]);
			wMean = [w0Mean, repmat(wi, 1, 2*N)];

			numSteps = length(obj.trajectory.time);

			% For adaptive Q matrix
            % https://www.sciencedirect.com/science/article/pii/S1566253520303286
            Q = diag(repmat(0.01, 1, N));

			% Recording Initialization
			xRec = zeros(N, numSteps - 2);
			PRec = zeros(N, N, numSteps - 2);
			Ptr = zeros(1, numSteps - 2);

			% Initialization
			L = sqrtm((N + lambda) * P0);  % '
			SP0 = [x0, repmat(x0, 1, N) + L, repmat(x0, 1, N) - L];  % sigma points
			dt = obj.trajectory.time(2) - obj.trajectory.time(1);  % delta time
			for ii=1:size(SP0, 2)
				SPm1(:,ii) = obj.f(SP0(:,ii), dt);
			end
			xm1 = SPm1 * wMean.';
			xRepmat = repmat(xm1, 1, 2*N+1);
			Pm1 = (SPm1 - xRepmat) * wCov * (SPm1 - xRepmat).' + Q;
			P = Pm1;
			x = xm1;

			% Kalman Filter
			for kk=1:numSteps-1

				% Update
				time = obj.trajectory.time(kk);  % current time
                Rarr = repmat(10E10, 1, 3*length(obj.cameras));
                Zmes = repmat([0; 0; 1], length(obj.cameras), 1);
                zSpace = zeros(3*length(obj.cameras), 2*N + 1);

				for camNum=1:length(obj.cameras)
					cam = obj.cameras(camNum);  % cam object
					timeIndex = find(cam.obsTimeSaw == time, 1);  % gets time index of measurement
                    zSpace(3*camNum - 2: 3*camNum, :) = cam.obsFun(SPm1(1:3, :), obj.config.GolfBall.radius);
					if ~isempty(timeIndex)  % checks if there is a measurement
						Rarr(3*camNum-2:3*camNum) = cam.mesUncertainty.^2;
                        Zmes(3*camNum-2:3*camNum) = cam.obsUVAmes(:, timeIndex);
                    end
				end

				R = diag(Rarr);  % convert array to diaginal matrix

				xRepmat = repmat(xm1, 1, 2*N+1);
				zBar = zSpace * wMean.';
				Pz = (zSpace - zBar) * wCov * (zSpace - zBar).' + R;
				Pxz = (SPm1 - xRepmat) * wCov * (zSpace - zBar).';
				K = Pxz / Pz;
				
				x = xm1 + K*(Zmes - zBar);
				P = Pm1 - K*Pz*K.';

				% Predict
				L = real(sqrtm((N + lambda) * P));
				SP = [x, repmat(x, 1, N) + L, repmat(x, 1, N) - L];  % sigma points
				for ii=1:size(SP0, 2)
					SPp1(:,ii) = obj.f(SP(:,ii), dt);
				end
				xp1 = SPp1 * wMean.';
				xp1Repmat = repmat(xp1, 1, 2*N+1);
				Pp1 = (SPp1 - xp1Repmat) * wCov * (SPp1 - xp1Repmat).' + Q;

				% Unit Delay
				xm1 = xp1;
				SPm1 = SPp1;
				Pm1 = Pp1;

				% Recording
				xRec(:, kk) = x;
				PRec(:, :, kk) = P;
				Ptr(kk) = trace(P(1:3,1:3));
			end
			obj.UKF.xRec = xRec;
			obj.UKF.PRec = PRec;
			obj.UKF.Ptr = Ptr;
		end
				% Adaptive Q
				%zObs = double.empty([0, 1]);
				%for camNum=1:length(obj.cameras)
				%	timeIndex = find(cam.obsTimeSaw == time, 1);  % gets time index of measurement
				%	cam = obj.cameras(camNum);  % cam object
				%	if ~isempty(timeIndex)  % checks if there is a measurement
				%		zObs = [zObs; cam.obsFun(xm1(1:3), obj.config.GolfBall.radius)];
				%	end
				%end
				%mu = Zmes - zObs;
				%phi = mu.'*inv(Pz + R)*mu;
				%if (phi > chi)
				%	lambda = max([lambda0, (phi - 5*chi)/phi]);
				%	Q = (1-lambda)*Q + lambda*K*mu*mu.'*K.';
				%	disp(Q)
				%end
		function drawResults(obj)
			org = Orgin(10, [0, 0, 0]);
			org.lineWidth = 2;
			pos = obj.trajectory.xTrue(:,1:3);
			numSteps = pos(:,2);
			
			figure('Position', [100 400 600 500]);
			view([0 0])
			axis vis3d off equal
			view([45 45])
			grid on
			rotate3d on
			hold on

			org.drawOrgin();
			for ii=1:length(obj.cameras)
				obj.cameras(ii).drawCam()
			end
			
			UKFpos = obj.UKF.xRec(1:3, :);
			UKFvel = obj.UKF.xRec(4:6, :);
			UKFrad = obj.UKF.xRec(7:9, :);

			plot3(pos(:,1),pos(:,2),pos(:,3), 'k-');
			plot3(UKFpos(1,:),UKFpos(2,:),UKFpos(3,:), 'r.');
			plot3(pos(1,1),pos(1,2),pos(1,3), 'r.', 'MarkerSize', 20);
			plot3(pos(:,1),pos(:,2), zeros(obj.trajectory.numSteps, 1), 'Color', [0.5 0.5 0.5]);
			
			for ii=1:length(obj.cameras)
				figure('Position', [100 100 400 300]);
				obj.cameras(ii).drawObs()
			end

			% UKF plots
			posError = UKFpos - obj.trajectory.xTrue(2:end, 1:3).';
			posErrorMag = vecnorm(posError,2,1);

			velError = UKFvel - obj.trajectory.xTrue(2:end, 4:6).';
			velErrorMag = vecnorm(velError,2,1);

			radError = UKFrad - obj.trajectory.xTrue(2:end, 7:9).';
			radErrorMag = vecnorm(radError,2,1);

			figure('Position', [1000 100 700 600]);
			subplot(3,1,1);
			semilogy(obj.trajectory.time(2:end), posErrorMag)
			xlabel('time')
			ylabel('error (m)')
			grid on

			subplot(3,1,2);
			semilogy(obj.trajectory.time(2:end), velErrorMag)
			xlabel('time')
			ylabel('error (m/s)')
			grid on

			subplot(3,1,3);
			semilogy(obj.trajectory.time(2:end), radErrorMag)
			xlabel('time')
			ylabel('error (rad/s)')
			grid on

			%sgtitle('Main Title for All Subplots');
		end

		function xNew = f(obj, x, dt)
			% RK4
			k1 = obj.motionOfBall(x);
			k2 = obj.motionOfBall(x + dt/2*k1);
			k3 = obj.motionOfBall(x + dt/2*k2);
			k4 = obj.motionOfBall(x + dt*k3);
			xNew = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
		end

        % Calculate the motion of the ball
        function dxdt = motionOfBall(obj, x)
            eps = 0.01;  % Small number to avoid division by zero

            % Retrieve parameters from configuration
            ballParms = obj.config.GolfBall;
            envParms = obj.config.Environment;

            % Ball and environment parameters
            m = ballParms.mass;
            R = ballParms.radius;
            tau = ballParms.spinDecay;
            A = pi * R^2;
            rho = envParms.airDensity;
            g = envParms.gravity;
            visk = envParms.viscosity;

            % Extract velocity and spin vectors from state
            vVec = x(4:6);
            v = norm(vVec);
            wVec = x(7:9);
            w = norm(wVec);
            gVec = [0; 0; -g];

            % Calculate coefficients
            S = w * R / (v + eps);
            Re = 2 * v * R / visk;
            Cl = obj.getCl(S);
            Cd = obj.getCd(Re);

            % Calculate force vectors
			% https://www.researchgate.net/publication/258303995_The_motion_of_an_arbitrarily_rotating_spherical_projectile_and_its_application_to_ball_games
            dragAcc = -1/(2 * m) * rho * A * Cd * v * vVec;
            liftAcc = 1/(2 * m) * rho * A * Cl * v * (cross(wVec, vVec) / (w + eps));

            % Equations of motion
            posDot = x(4:6);
            velDot = dragAcc + liftAcc + gVec;
            radDot = -tau * wVec;

            dxdt(1:3, 1) = posDot;
            dxdt(4:6, 1) = velDot;
            dxdt(7:9, 1) = radDot;
        end

        % Calculate lift coefficient (Cl)
        function Cl = getCl(obj, S)
			% https://www.mdpi.com/2504-3900/2/6/238
			% https://www.desmos.com/calculator/lct560dzqp
            a = 0.33;
            b = 7.6;
            Cl = a * (1 - exp(-b * S));
        end
        
        % Calculate drag coefficient (Cd)
        function Cd = getCd(obj, Re)
			% https://www.mdpi.com/2504-3900/2/6/238
			% https://www.desmos.com/calculator/pk2175tmgn
            lowRe = 0;
            midRe = 81207;
            highRe = 2E5;
            
            % Select the appropriate Cd calculation based on Re
            if Re < midRe
                Cd = obj.getCdLow(Re);
            elseif Re >= midRe && Re < highRe
                Cd = obj.getCdHigh(Re);
            else
                Cd = obj.getCdHigh(highRe);
            end
        end
    end
    
    methods (Static)
        % Event function to detect when the ball hits the ground
        function [value, isterminal, direction] = groundEvent(t, y)
            value = y(3);        % Detect when height = 0
            isterminal = 1;      % Stop the integration
            direction = -1;      % Detect when height is decreasing
        end

        % Calculate low Reynolds number Cd
        function Cd = getCdLow(Re)
            Cd = 1.29E-10 * Re^2 - 2.59E-5 * Re + 1.5;
        end
        
        % Calculate high Reynolds number Cd
        function Cd = getCdHigh(Re)
            Cd = 1.91E-11 * Re^2 - 5.40E-6 * Re + 0.56;
        end

        % Load configuration from JSON file
        function config = getConfigStruct(configName)
            % Open the JSON file
            fid = fopen(configName, 'r');
            
            % Read the file's content
            rawJson = fread(fid, inf, 'char');
            fclose(fid);
            
            % Convert the raw content to a string and decode
            jsonStr = char(rawJson');
            config = jsondecode(jsonStr);
        end
    end
end
