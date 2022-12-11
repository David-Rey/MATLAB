
close all; clear; clc;

%P = [4 5; 5 1];

%[V,D] = eig(P);

%vec1 = V(:,1) * D(1,1);
%vec2 = V(:,2) * D(2,2);

V = createVmatrix(0);
D = diag([2 1]);
P = V*D*inv(V);
disp(P)
disp(trace(P))
disp(det(P))
% YES

figure;
axis tight equal
grid on
set(gcf,'Position',[800 100 650 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold on

%plot([0, vec1(1)], [0, vec1(2)],'LineWidth',2);
%plot([0, vec2(1)], [0, vec2(2)],'LineWidth',2);

function V = createVmatrix(theta)
    vec1 = [cosd(theta); sind(theta)];
    vec2 = [cosd(theta+90); sind(theta+90)];
    V = [vec1 vec2];
end
