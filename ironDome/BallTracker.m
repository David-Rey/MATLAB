classdef BallTracker < handle
    properties
        trajectory;     % Trajectory struct
        config;         % Configuration parameters
		cameras;        % Camera objects
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
			end
		end

        % Run the simulation
        function runSim(obj)
			obj.simTrajectory()
			obj.getCamObs()
			%obj.runEKF()
		end

		function simTrajectory(obj)
			dt = obj.config.SimSettings.dt;
            tspan = 0:dt:obj.config.SimSettings.maxTime;  % Time span for the simulation

            % Set options for ODE solver, including event function
            options = odeset('Events', @obj.groundEvent);
            
            % Solve the ODE
            [time, xTrue] = ode45(@(t, x) obj.motionOfBall(t, x), tspan, obj.trajectory.x0, options);
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

		function runEKF(obj)

			syms x [1 3] real;
			syms s [numSensors 3] real;

			I = eye(9);
			Q = I*.02;
			
			% Kalman Filter
			for kk=1:numSteps - 1
				currectMes = noisyObs(:,kk); % current observation
   			
				PPre = PRec(:,:,kk); % previous covarance matrix
				xPre = xRecEst(:,kk); % previous state vector
			
    			H = fH(xPre,obsPos); % observation matrix
    			K = PPre*H.'*inv(H*PPre*H.' + R); % kalman gain
    			xEst = xPre + K*(currectMes - obsFunc(xPre)); % state estimate
    			PEst = (I - K*H)*PPre*(I - K*H).' + K*R*K.'; % covarance update
			
    			dt = t(kk+1) - t(kk);
    			A = Jfun(xEst); % continous system matrix
    			b = Bfun(xEst); % continuous b vector
    			F = expm(A*dt); % discrete time system matrix
			
    			xNxt = F*xEst + b*dt; % next state
				PNxt = F*PEst*F.' + Q; % next covarance matrix
			end
		end

		function drawResults(obj)
			org = Orgin(10, [0, 0, 0]);
			org.lineWidth = 2;
			pos = obj.trajectory.xTrue(:,1:3);
			numSteps = pos(:,2);
	
			view([0 0])
			axis vis3d off
			view([45 45])
			grid on
			%rotate3d on
			hold on

			org.drawOrgin();
			for ii=1:length(obj.cameras)
				obj.cameras(ii).drawCam()
			end
			
			plot3(pos(:,1),pos(:,2),pos(:,3), 'k-');
			plot3(pos(1,1),pos(1,2),pos(1,3), 'r.', 'MarkerSize', 20);
			plot3(pos(:,1),pos(:,2), repmat(0, obj.trajectory.numSteps, 1), 'Color', [0.5 0.5 0.5]);
			
			for ii=1:length(obj.cameras)
				obj.cameras(ii).drawObs()
			end
		end

        % Calculate the motion of the ball
        function dxdt = motionOfBall(obj, t, x)
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
            S = w * R / v;
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
            radDot = repmat(tau, 3, 1);

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
