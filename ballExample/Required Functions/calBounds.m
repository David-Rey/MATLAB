function [xmin,xmax,ymin,ymax] = calBounds(sens,r_x)
	axisBuffer = 50; 	% set axis limits
	[xmin, ymin, xmax, ymax] = deal(0,0,0,0);
	for ii=1:size(sens,1)
		xmin = min(sens(ii,1),xmin);
		xmax = max(sens(ii,1),xmax);
		ymin = min(sens(ii,2),ymin);
		ymax = max(sens(ii,2),ymax);
	end
	xmin = min(min(r_x(1,:)),xmin)-axisBuffer;
	xmax = max(max(r_x(1,:)),xmax)+axisBuffer;
	ymin = min(min(r_x(2,:)),ymin)-axisBuffer;
	ymax = max(max(r_x(2,:)),ymax)+axisBuffer;
	xlim([xmin,xmax]);
	ylim([ymin,ymax]);
end
