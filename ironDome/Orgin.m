classdef Orgin < handle
	properties
		bVec (3,1) double {mustBeReal}
		rVec (3,1) double {mustBeReal}
		lineWidth = 1;
		scaleFactor = 1;
		colorSat = 0;
	end
	properties (Access = private)
		Rf = eye(3);
		Rr = eye(3);
		xTip;
		yTip;
		zTip;
		orgLines
		orgPoint
	end
	
	methods
		function obj = Orgin(scaleFactor, bVec)
			if nargin == 1
				obj.scaleFactor = scaleFactor;
				obj.bVec = [0;0;0];
			end
			if nargin == 2
				obj.scaleFactor = scaleFactor;
				obj.bVec = bVec;
			end
			obj.xTip = obj.bVec' + [obj.scaleFactor; 0; 0];
			obj.yTip = obj.bVec' + [0; obj.scaleFactor; 0];
			obj.zTip = obj.bVec' + [0; 0; obj.scaleFactor];
		end
		
		function setLineWidth(obj, inWidth)
			obj.lineWidth = inWidth;
		end
		
		function setTranslation(obj, pos)
			obj.bVec = pos;
		end
		
		function [Rx,Ry,Rz] = getRotationMatrices(obj)
			x = obj.rVec(1);
			y = obj.rVec(2);
			z = obj.rVec(3);
			Rx = [1,0,0; 0,cosd(x),-sind(x); 0,sind(x),cosd(x)]; % x rotation matrix
			Ry = [cosd(y),0,sind(y); 0,1,0; -sind(y),0,cosd(y)]; % y rotation matrix
			Rz = [cosd(z),-sind(z),0; sind(z),cosd(z),0; 0,0,1]; % z rotation matrix
		end
		
		function setRotation(obj, rVec)
			obj.rVec = rVec;
			obj.Rf = getRotationForward(obj);
			obj.Rr = getRotationReverse(obj);
		end
		
		function Rf = getRotationForward(obj)
			[Rx,Ry,Rz] = getRotationMatrices(obj);
			Rf = Rz*Ry*Rx; % rotation matrix in zyx (order matters in matrix multiplication!)
		end
		
		function Rr = getRotationReverse(obj)
			[Rx,Ry,Rz] = getRotationMatrices(obj);
			Rr = Rx*Ry*Rz; % rotation matrix in xyz (order matters in matrix multiplication!)
		end
		
		function transformOrgForward(obj)
			obj.xTip = obj.Rf*[obj.scaleFactor;0;0] + obj.bVec;
			obj.yTip = obj.Rf*[0;obj.scaleFactor;0] + obj.bVec;
			obj.zTip = obj.Rf*[0;0;obj.scaleFactor] + obj.bVec;
		end
		
		function drawOrgin(obj)
			transformOrgForward(obj);
			
			delete(obj.orgLines);
			delete(obj.orgPoint);
			
			obj.orgLines = gobjects(0);

			width = obj.lineWidth;
			sat = obj.colorSat;
			
			pos = obj.bVec;
			x = obj.xTip;
			y = obj.yTip;
			z = obj.zTip;
			
			obj.orgLines(1) = line([pos(1),x(1)],[pos(2),x(2)],[pos(3),x(3)],'color',max(sat,[1 0 0]),'lineWidth',width);
			obj.orgLines(2) = line([pos(1),y(1)],[pos(2),y(2)],[pos(3),y(3)],'color',max(sat,[0 1 0]),'lineWidth',width);
			obj.orgLines(3) = line([pos(1),z(1)],[pos(2),z(2)],[pos(3),z(3)],'color',max(sat,[0 0 1]),'lineWidth',width);
		end
	end
end