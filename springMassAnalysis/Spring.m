classdef Spring < handle
    properties
        p1x {mustBeReal}
        p1y {mustBeReal}
        p2x {mustBeReal}
        p2y {mustBeReal}
        springH {mustBeReal}
        numSpokes {mustBeReal}
    end
    properties (Access = private)
        vector;
        unitVector;
        lineArr;
        pointArr;
        deltaWidth; % vector with length between spokes
        Rccw = [0 -1; 1 0];
        Rcw = [0 1; -1 0];
    end
    methods
        function obj = Spring(x1,y1,x2,y2,h,numS)
            obj.p1x = x1;
            obj.p1y = y1;
            obj.p2x = x2;
            obj.p2y = y2;
            obj.springH = h;
            obj.numSpokes = numS;
            
            obj.updateInternalVars()

            for ii=1:size(obj.pointArr,2) - 1
                obj.lineArr(ii) = line([obj.pointArr(1,ii),obj.pointArr(1,ii+1)], [obj.pointArr(2,ii),obj.pointArr(2,ii+1)]);
            end
        end
        
        function updateInternalVars(obj)
            obj.vector = [obj.p2x;obj.p2y] - [obj.p1x;obj.p1y];
            obj.unitVector = obj.vector / norm(obj.vector);
            obj.deltaWidth = obj.vector / (obj.numSpokes + 1);
            obj.pointArr(:,1) = [obj.p1x;obj.p1y];

            for ii=1:obj.numSpokes
                fracAlongVec = (2*ii - 1) / (2 * obj.numSpokes);
                pointAlongVec = fracAlongVec * obj.vector;
                if mod(ii,2) == 0
                    unitPerp = obj.Rccw*obj.unitVector;
                else
                    unitPerp = obj.Rcw*obj.unitVector;
                end
                perpVec = unitPerp * obj.springH;
                point = pointAlongVec + perpVec + [obj.p1x;obj.p1y];
                obj.pointArr(:,ii+1) = point;
            end
            obj.pointArr(:,obj.numSpokes+2) = [obj.p2x;obj.p2y];
        end

        function drawSpring(obj)
            obj.updateInternalVars()
            for ii=1:length(obj.lineArr)
                set(obj.lineArr(ii),'XData',[obj.pointArr(1,ii), obj.pointArr(1,ii+1)]);
                set(obj.lineArr(ii),'YData',[obj.pointArr(2,ii), obj.pointArr(2,ii+1)]);
            end
        end

        function setLineWidth(obj, width)
            for ii=1:length(obj.lineArr)
                set(obj.lineArr(ii), 'LineWidth', width);
            end
        end
         
        function setLineColor(obj, color)
            for ii=1:length(obj.lineArr)
                set(obj.lineArr(ii), 'color', color);
            end
        end

        function printPointArr(obj)
            disp(obj.pointArr)
        end

    end
end