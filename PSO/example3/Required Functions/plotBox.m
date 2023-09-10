function plotBox(boxBoundX,boxBoundY,boxBoundZ,boxColor)
	% Left side box
	line([boxBoundX(1),boxBoundX(1)], [boxBoundY(1),boxBoundY(2)], [boxBoundZ(1),boxBoundZ(1)],'Color',boxColor);
	line([boxBoundX(1),boxBoundX(1)], [boxBoundY(1),boxBoundY(2)], [boxBoundZ(2),boxBoundZ(2)],'Color',boxColor);
	line([boxBoundX(1),boxBoundX(1)], [boxBoundY(1),boxBoundY(1)], [boxBoundZ(1),boxBoundZ(2)],'Color',boxColor);
	line([boxBoundX(1),boxBoundX(1)], [boxBoundY(2),boxBoundY(2)], [boxBoundZ(1),boxBoundZ(2)],'Color',boxColor);
	% Right side box
	line([boxBoundX(2),boxBoundX(2)], [boxBoundY(1),boxBoundY(2)], [boxBoundZ(1),boxBoundZ(1)],'Color',boxColor);
	line([boxBoundX(2),boxBoundX(2)], [boxBoundY(1),boxBoundY(2)], [boxBoundZ(2),boxBoundZ(2)],'Color',boxColor);
	line([boxBoundX(2),boxBoundX(2)], [boxBoundY(1),boxBoundY(1)], [boxBoundZ(1),boxBoundZ(2)],'Color',boxColor);
	line([boxBoundX(2),boxBoundX(2)], [boxBoundY(2),boxBoundY(2)], [boxBoundZ(1),boxBoundZ(2)],'Color',boxColor);
	% Top and bottom side box
	line([boxBoundX(1),boxBoundX(2)], [boxBoundY(1),boxBoundY(1)], [boxBoundZ(1),boxBoundZ(1)],'Color',boxColor);
	line([boxBoundX(1),boxBoundX(2)], [boxBoundY(1),boxBoundY(1)], [boxBoundZ(2),boxBoundZ(2)],'Color',boxColor);
	line([boxBoundX(1),boxBoundX(2)], [boxBoundY(2),boxBoundY(2)], [boxBoundZ(1),boxBoundZ(1)],'Color',boxColor);
	line([boxBoundX(1),boxBoundX(2)], [boxBoundY(2),boxBoundY(2)], [boxBoundZ(2),boxBoundZ(2)],'Color',boxColor);
	% orgin axis
	line([0, boxBoundX(2)],[0, 0],[0, 0],'LineStyle',':','Color',boxColor);
	line([0, 0],[0, boxBoundY(2)],[0, 0],'LineStyle',':','Color',boxColor);
	line([0, 0],[0, 0],[0, boxBoundZ(2)],'LineStyle',':','Color',boxColor);
	plot3(0,0,0,'k*'); % plots orgin
	line([0,0],[0,0],[boxBoundZ(1),0],'Color',[.8 .8 .8]);
end