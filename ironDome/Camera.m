classdef Camera < handle
	properties
		bVec (3,1) double {mustBeReal}
		rotVec (1,3) double {mustBeReal}
		fov (1,2) double {mustBeReal}
		depth {mustBeReal}
		sensorColor = 'k';
		fovColor = [.5 .5 .5];
		showFrontCross logical
		showFrontMarker logical
		maxUV (1,2) double {mustBeReal}
	end
	properties (Access = private)
		Rf = eye(3);
		Rr = eye(3);
		staticFrame;
		frame;
		frameGeo;
		orginGeo;
	end
	
	methods
		function obj = Camera(FOV, depth)
			obj.fov = FOV;
			obj.depth = sort(depth);
			obj.showFrontCross = 1;
			obj.showFrontMarker = 1;
			obj.maxUV = [tand(FOV(1)/2), tand(FOV(2)/2)];
			setUpFrame(obj);
		end
		
		function setRotation(obj, rVec)
			obj.rotVec = rVec;
			obj.Rf = getRotationForward(obj);
			obj.Rr = getRotationReverse(obj);
		end
		
		function setTranslation(obj, bVec)
			obj.bVec = bVec;
		end
		
		function logical = isInView(obj, point)
			%tPoint = transformPointReverse(obj,point);
			tPoint = inv(obj.Rr)*(point - obj.bVec);
			thx = (90 - (obj.fov(1)/2));
			thy = (90 - (obj.fov(2)/2));
			
			[thPtX,~] = cart2pol(tPoint(1,:),tPoint(3,:));
			thPtX = rad2deg(thPtX);
			[thPtY,~] = cart2pol(tPoint(2,:),tPoint(3,:));
			thPtY = rad2deg(thPtY);
			rPt = vecnorm(tPoint);
			
			checkX = thx < thPtX & 180-thx > thPtX;
			checkY = thy < thPtY & 180-thy > thPtY;
			checkR = rPt > obj.depth(1) & rPt < obj.depth(end);
			
			logical = checkX & checkY & checkR;
		end
		
		function [Rx,Ry,Rz] = getRotationMatrices(obj)
			x = obj.rotVec(1);
			y = obj.rotVec(2);
			z = obj.rotVec(3);
			Rx = [1,0,0; 0,cosd(x),-sind(x); 0,sind(x),cosd(x)]; % x rotation matrix
			Ry = [cosd(y),0,sind(y); 0,1,0; -sind(y),0,cosd(y)]; % y rotation matrix
			Rz = [cosd(z),-sind(z),0; sind(z),cosd(z),0; 0,0,1]; % z rotation matrix
		end
		
		function Rf = getRotationForward(obj)
			[Rx,Ry,Rz] = getRotationMatrices(obj);
			Rf = Rz*Ry*Rx; % rotation matrix in zyx (order matters in matrix multiplication!)
		end
		
		function Rr = getRotationReverse(obj)
			[Rx,Ry,Rz] = getRotationMatrices(obj);
			Rr = Rx*Ry*Rz; % rotation matrix in xyz (order matters in matrix multiplication!)
		end
		
		function setFrontCross(obj, logical)
			obj.showFrontCross = logical;
			clearFrame(obj);
			setUpFrame(obj);
		end

		function setFrontMarker(obj, logical)
			obj.showFrontMarker = logical;
			clearFrame(obj);
			setUpFrame(obj);
		end
		
		function setFOV(obj, FOV)
			obj.fov = FOV;
			clearFrame(obj);
			setUpFrame(obj);
		end

		function setDepth(obj, depth)
			obj.depth = sort(depth);
			clearFrame(obj);
			setUpFrame(obj);
		end

		function tPoint = transformPointReverse(obj, point)
			tPoint = obj.Rr*(point - obj.bVec);
		end
		
		function setUpFrame(obj)
			tempFrame.points = [0;0;0;];
			tempFrame.type = '';
			tempFrame.color = [0 0 0];
			
			for iRange=1:length(obj.depth)
				for iFOV=-1:2:1
					for funToUse=1:2
						switch funToUse
							case 1
								tempFrame(end+1).points = getFOVarcX(obj,iFOV*obj.fov(2),obj.depth(iRange));
							case 2
								tempFrame(end+1).points = getFOVarcY(obj,iFOV*obj.fov(1),obj.depth(iRange));
						end
						switch iRange
							case 1
								tempFrame(end).type = 'closeFeild';
							case length(obj.depth)
								tempFrame(end).type = 'farFeild';
							otherwise
								tempFrame(end).type = 'interFeild';
						end
						tempFrame(end).color = obj.fovColor;
					end
				end
			end
			
			cPoints(:,1) = tempFrame(end-3).points(:,1);
			cPoints(:,2) = tempFrame(end-3).points(:,100);
			cPoints(:,3) = tempFrame(end-1).points(:,1);
			cPoints(:,4) = tempFrame(end-1).points(:,100);
			
			for ii=1:4
				tempFrame(end+1).points = [0,cPoints(1,ii); 0,cPoints(2,ii); 0,cPoints(3,ii)];
				tempFrame(end).type = 'edge';
				tempFrame(end).color = obj.fovColor;
			end
			
			if obj.showFrontMarker
				xDis = norm(cPoints(:,2) - cPoints(:,3)) / 4;
				yDis = norm(cPoints(:,1) - cPoints(:,3)) / 4;

				tempFrame(end+1).points = [0,xDis; 0,0; obj.depth(end),obj.depth(end)];
				tempFrame(end).type = 'Xcross';
				tempFrame(end).color = [1 0 0];
				tempFrame(end+1).points = [0,0; 0,yDis; obj.depth(end),obj.depth(end)];
				tempFrame(end).type = 'Ycross';
				tempFrame(end).color = [0 1 0];
			end
			
			if obj.showFrontCross
				tempFrame(end+1).points = getFOVarcX(obj,0,obj.depth(end));
				tempFrame(end).type = 'frontCross';
				tempFrame(end).color = obj.fovColor;
				
				tempFrame(end+1).points = getFOVarcY(obj,0,obj.depth(end));
				tempFrame(end).type = 'frontCross';
				tempFrame(end).color = obj.fovColor;
			end
			obj.staticFrame = tempFrame;
		end

		function clearFrame(obj)
			obj.staticFrame = rmfield(obj.staticFrame,fields(obj.staticFrame));
		end

		function points = getFOVarcX(obj,alpha,r)
			thx = deg2rad(90 - (obj.fov(1)/2));
			thy = deg2rad(90 - (alpha/2));
			cosX = cos(thx)^2;
			cosY = cos(thy)^2;
			maxL = sqrt((r^2*cosX*(1-cosY)) / (1-cosY*cosX)); % magic lol
			
			x = linspace(-maxL,maxL);
			rPrime = sqrt(r.^2 - x.^2);
			[y,z] = pol2cart(thy,rPrime);
			points = [x;y;z];
		end
		
		function points = getFOVarcY(obj,alpha,r)
			thx = deg2rad(90 - (alpha/2));
			thy = deg2rad(90 - (obj.fov(2)/2));
			cosX = cos(thx)^2;
			cosY = cos(thy)^2;
			maxL = sqrt((r^2*cosX*(1-cosY)) / (1-cosY*cosX)); % magic lol
			maxW = sqrt(r^2-maxL^2)*cos(thy);
			
			y = linspace(maxW,-maxW);
			rPrime = sqrt(r.^2 - y.^2);
			[x,z] = pol2cart(thx,rPrime);
			points = [x;y;z];
		end
		
		function transformFrameForward(obj)
			tempFrame = obj.staticFrame;
			for frameElement=1:length(obj.staticFrame)
				for ii=1:length(obj.staticFrame(frameElement).points(1,:))
					tempFrame(frameElement).points(:,ii) = obj.Rf*obj.staticFrame(frameElement).points(:,ii) + obj.bVec;
					tempFrame(frameElement).type = obj.staticFrame(frameElement).type;
					tempFrame(frameElement).color = obj.staticFrame(frameElement).color;
				end
			end
			obj.frame = tempFrame;
		end
		
		function tPoints = transformPointsReverse(obj,points)
			tPoints = zeros(size(points));
			for ii=1:length(points(1,:))
				tPoints(:,ii) = obj.Rr*(points(:,ii) - obj.bVec);
			end
		end

		function tPoints = transformPointsForward(obj,points)
			tPoints = zeros(size(points));
			for ii=1:length(points(1,:))
				tPoints(:,ii) = obj.Rf*points(:,ii) + obj.bVec;
			end
		end

		function loc = frameLocPoints(obj,points)
			numPoints = length(points(1,:))
			tPoints = inv(obj.Rf)*(points - repmat(obj.bVec, 1, numPoints));
			loc = zeros(size(tPoints));
			for ii=1:numPoints
				loc(1,ii) = -tPoints(1,ii) / tPoints(3,ii);  % u=x/z
				loc(2,ii) = tPoints(2,ii) / tPoints(3,ii);   % v=y/z
				loc(3,ii) = norm(tPoints(:,ii));             % r=norm(tPoint)
			end
		end
		
		function drawCam(obj)
			transformFrameForward(obj);
			
			delete(obj.frameGeo);
			delete(obj.orginGeo);

			obj.orginGeo = plot3(obj.bVec(1), obj.bVec(2), obj.bVec(3),...
				'h','MarkerSize', 10,'MarkerFaceColor','#EDB120','MarkerEdgeColor','red');
			
			obj.frameGeo = gobjects(0);
			for frameElement=1:length(obj.frame)
				x = obj.frame(frameElement).points(1,:);
				y = obj.frame(frameElement).points(2,:);
				z = obj.frame(frameElement).points(3,:);
				color = obj.frame(frameElement).color;
				obj.frameGeo(end+1) = line(x,y,z,'color',color);
			end
		end
	end
end
