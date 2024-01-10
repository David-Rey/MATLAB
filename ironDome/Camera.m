classdef Camera < handle
    % Camera Class for simulating a camera in 3D space.
    % This class handles camera properties such as position, orientation,
    % field of view, and frustum calculation for 3D visualization.

    properties
        bVec (3,1) double {mustBeReal}   % Camera position vector
        fov (1,2) double {mustBeReal}    % Field of view (degrees) in x and y directions
        maxUV (1,2) double {mustBeReal}  % Maximum UV coordinates (based on FOV)
        depth {mustBeReal}               % Depth of the view frustum
        az double {mustBeReal}           % Azimuth angle for rotation
        el double {mustBeReal}           % Elevation angle for rotation
        roll double {mustBeReal}         % Roll angle for rotation
        R = eye(3)                       % Rotation matrix (initialized to identity)
		obsUVAtru						 % True ovservations in UVA [ u=x/y, v=z/y, alpha = 2*atan(R/norm(pos) ]
		obsUVAmes						 % Estimate ovservations in UVA [ u=x/y, v=z/y, alpha = 2*atan(R/norm(pos) ]
		obsTimeSaw                       % Array of time elements
		mesUncertainty					 % Mesurment Uncertainty of UV
		name
    end

    properties (Access = private)
        staticFrustum;                   % Static frustum corners (before rotation and translation)
        frustumGeo;                      % Graphical objects for frustum visualization
        orginGeo;                        % Graphical object for camera position visualization
    end

    methods
        function obj = Camera(FOV, depth)
            % Constructor for Camera class.
            obj.fov = FOV;
            obj.depth = depth;
            obj.maxUV = [tand(FOV(1)/2), tand(FOV(2)/2)];
            obj.setUpFrustum();
        end

        function setTranslation(obj, bVec)
            % Set the translation (position) of the camera.
            obj.bVec = bVec;
        end

        function setRotation(obj, az, el, roll)
            % Set the rotation of the camera.
            obj.az = az;
            obj.el = el;
            obj.roll = roll;
            [Rx,Ry,Rz] = obj.getRotationMatrices();
            obj.R = Rz*Rx*Ry; % Update rotation matrix
        end

        function [Rx,Ry,Rz] = getRotationMatrices(obj)
            % Calculate rotation matrices for each axis.
            x = obj.el;
            y = obj.roll;
            z = -obj.az;
            Rx = [1,0,0; 0,cosd(x),-sind(x); 0,sind(x),cosd(x)]; % x rotation
            Ry = [cosd(y),0,sind(y); 0,1,0; -sind(y),0,cosd(y)]; % y rotation
            Rz = [cosd(z),-sind(z),0; sind(z),cosd(z),0; 0,0,1]; % z rotation
		end

		function tPoints = global2local(obj, points)
        	% Transform points from the global (world) coordinate system to the local (camera) coordinate system.
	
        	% Ensure points are in a column-wise format
        	if size(points, 1) ~= 3
            	points = points';
        	end
	        numPoints = size(points, 2);
        	translatedPoints = points - repmat(obj.bVec, 1, numPoints);
        	tPoints = obj.R' * translatedPoints;
		end

		function bools = isInView(obj, points)
    		% Initialize the boolean array
    		bools = false(1, size(points, 2));  % One entry per point
		
    		% Calculate half-angles for easier computation
    		halfHFOV = obj.fov(1) / 2;
    		halfVFOV = obj.fov(2) / 2;
		
    		% Iterate over each point
    		for i = 1:size(points, 2)
        		% Normalize the point to get direction vector
        		directionVector = points(:, i) / norm(points(:, i));
		
        		% Calculate angles from direction vector
        		angleH = atand(directionVector(1) / directionVector(2));
				logicH = -halfHFOV < angleH && halfHFOV > angleH;

				angleV = atand(directionVector(3) / directionVector(2));
				logicV = -halfVFOV < angleV && halfVFOV > angleV;
		
        		% Check if point is within FOV
        		bools(i) = logicH & logicV;
    		end
		end


    	function tPoints = local2global(obj, points)
        	% Transform points from the local (camera) coordinate system to the global (world) coordinate system.
	
        	% Ensure points are in a column-wise format
        	if size(points, 1) ~= 3
            	points = points';
        	end
	
        	rotatedPoints = obj.R * points;
        	tPoints = rotatedPoints + obj.bVec;
    	end

		function obs = obsFun(obj, points, ballRadius)
			if size(points, 1) ~= 3
            	points = points';
			end
			tPoints = obj.global2local(points);
			numPoints = size(points, 2);
        	% Calculate the (u, v) coordinates on the image plane and distance r
        	obs(1, :) = tPoints(1, :) ./ tPoints(2, :);  % u = x/y
        	obs(2, :) = tPoints(3, :) ./ tPoints(2, :);  % v = z/y
			obs(3, :) = 2*atan(ballRadius ./ vecnorm(tPoints));
		end

		function global2UVA(obj, points, time, ballRadius)
			if size(points, 1) ~= 3
            	points = points';
			end
	        % Apply rotation and translation transformations
        	transformedPointsAll = obj.global2local(points);
			bools = isInView(obj, transformedPointsAll);
			transformedPoints = transformedPointsAll(:, bools);
			filteredTime = time(bools);
	
        	% Initialize the location matrix
        	obs = zeros(3, length(filteredTime));
	
        	% Calculate the (u, v) coordinates on the image plane and distance r
        	obs(1, :) = transformedPoints(1, :) ./ transformedPoints(2, :);  % u = x/y
        	obs(2, :) = transformedPoints(3, :) ./ transformedPoints(2, :);  % v = z/y
			obs(3, :) = 2*atan(ballRadius ./ vecnorm(transformedPoints));
			obj.obsUVAtru = obs;
			obj.obsTimeSaw = filteredTime;
		end

		function addCamNoise(obj)
			stdU = 0.03;
			stdV = 0.03;
			stdAlpha = .001;
			obj.mesUncertainty = [stdU stdV stdAlpha];
			Ru = diag(obj.mesUncertainty.^2);
			obj.obsUVAmes = Ru*randn(size(obj.obsUVAtru)) + obj.obsUVAtru;
		end

        function setUpFrustum(obj)
            % Calculate and set up the static frustum corners.
            halfFovX = obj.fov(1) / 2;
            halfFovY = obj.fov(2) / 2;
            halfWidth = obj.depth * tand(halfFovX);
            halfHeight = obj.depth * tand(halfFovY);
            farTopLeft     = [-halfWidth; obj.depth; halfHeight];
            farTopRight    = [halfWidth; obj.depth; halfHeight];
            farBottomLeft  = [-halfWidth; obj.depth; -halfHeight];
            farBottomRight = [halfWidth; obj.depth; -halfHeight];
            obj.staticFrustum = [farTopLeft, farTopRight, farBottomLeft, farBottomRight];
        end

		function drawCam(obj)
    		% Visualize the camera and its frustum.
    		delete(obj.frustumGeo);
    		delete(obj.orginGeo);
		
    		% Draw the camera position
    		obj.orginGeo = plot3(obj.bVec(1), obj.bVec(2), obj.bVec(3), ...
                         		'h', 'MarkerSize', 10, 'MarkerFaceColor', '#EDB120', 'MarkerEdgeColor', 'red');
		
    		% Update dynamic frustum based on rotation and translation
    		transformedFrustum = obj.R * obj.staticFrustum + obj.bVec;
		
    		% Drawing lines for the frustum
    		obj.frustumGeo = gobjects(8, 1); % 8 lines for the frustum
    		for i = 1:4
        		% Lines from camera to frustum corners
        		frustumPoint = transformedFrustum(:, i);
        		obj.frustumGeo(i) = line([obj.bVec(1), frustumPoint(1)], ...
                                 		[obj.bVec(2), frustumPoint(2)], ...
                                 		[obj.bVec(3), frustumPoint(3)], ...
                                 		'Color', [0, 0, 0]); % Draw frustum lines
    		end
		
    		% Define lines connecting the corners of the frustum far plane
    		cornerLines = [1,2; 2,4; 4,3; 3,1];
    		for i = 1:size(cornerLines, 1)
        		startPoint = transformedFrustum(:, cornerLines(i, 1));
        		endPoint = transformedFrustum(:, cornerLines(i, 2));
        		obj.frustumGeo(i+4) = line([startPoint(1), endPoint(1)], ...
                                   		[startPoint(2), endPoint(2)], ...
                                   		[startPoint(3), endPoint(3)], ...
                                   		'Color', [0, 0, 0]);
    		end
		end

		function drawObs(obj)
			
			axis equal
			hold on

			truObs = obj.obsUVAtru;
			mesObs = obj.obsUVAmes;
			maxU = obj.maxUV(1);
			maxV = obj.maxUV(2);

			plot(truObs(1, :), truObs(2, :), 'k');
			plot(mesObs(1, :), mesObs(2, :), 'Color', [0.5 0.5 0.5]);
			plot([-maxU, maxU, maxU, -maxU, -maxU], [maxV, maxV, -maxV, -maxV, maxV], 'k--');
			crosshairScale = 0.3;
			plot([0, 0], [-maxV * crosshairScale, maxV * crosshairScale], 'b-');
			plot([-maxU * crosshairScale, maxU * crosshairScale], [0, 0], 'b-');
			title(obj.name)
		end
    end
end
