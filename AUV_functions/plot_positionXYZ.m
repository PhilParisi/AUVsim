function [] = plot_positionXYZ(Data)
% Plot Position XYZ in 3D

figure()

plot3(Data.xyz(1,:),Data.xyz(2,:),Data.xyz(3,:),...
    '.k'), hold on
plot3(Data.xyz(1,1), Data.xyz(2,1), Data.xyz(3,1),...
    'ko','MarkerSize',10,'MarkerFaceColor','g');
plot3(Data.xyz(1,end), Data.xyz(2,end), Data.xyz(3,end),...
    'ko','MarkerSize',10,'MarkerFaceColor','r');

grid on
title('Position X, Y, Z (Earth Frame)')
set(gca,'ZDir','Rev')
xlabel('X_{North} [m]')
ylabel('Y_{East} [m]')
zlabel('Z_{Depth} [m]')
legend('Path','Start','End')
hold off

end