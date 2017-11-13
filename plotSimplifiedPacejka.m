% plotSimplifedPacejka
c

slipAngle = linspace(-15,15,51)*myConstants.deg2rad;
slipRatio = linspace(-0.2,0.2,51);
[SA SR] = meshgrid(slipAngle,slipRatio);
Fz = 2000;

for i = 1:length(slipAngle)
    for j = 1:length(slipRatio)
        [fx, fy] = simplifiedPacejka(Fz,slipAngle(i),slipRatio(j));
        Fx(i,j) = fx;
        Fy(i,j) = fy;
        fr = rms([fx,fy]);
        Fr(i,j) = fr;
        muR(i,j) = fr/Fz;
    end
end

%%
figure
[C,h] = contour(-SR,SA*myConstants.rad2deg,Fr/1000,'linewidth',1.5);
xlabel('Slip Angle')
ylabel('Slip Ratio')
xlim([-15 15])
ylim([-0.2 0.2])
grid on



v = [1 1.8  2.0 2.5 2.9  3.3 3.5 3.57];
clabel(C,h,v)


%%
figure
surf(SA*myConstants.rad2deg,-SR,Fr/1000,'linewidth',1.5);
xlabel('Slip Angle')
ylabel('Slip Ratio')
xlim([-15 15])
ylim([-0.2 0.2])
grid on

%%

% figure
% Fz = 1000;
% slipRatio = linspace(-0.2,0.2,51);
% slipAngle = 0;
% for i = 1:5
%     for j = 1:length(slipRatio)
%         [fx,fy] = simplifiedPacejka(Fz,slipAngle,slipRatio(j));
%         Fx(j) = fx;
%     end
%     plot(-slipRatio,Fx/1000)
%     hold all
%     Fz = Fz + 1000;
% end
% grid on
    

%%
% figure
% Fz = 2000;
% slipAngle = 0;
% 
% slipRatio = linspace(-0.2,0.2,51);
% 
% for i = 1:5
%     for j = 1:length(slipRatio)
%         [fx,fy] = simplifiedPacejka(Fz,slipAngle,slipRatio(j));
%         Fx(j) = fx;
%     end
%     plot(-slipRatio,Fx/1000)
%     hold all
%     slipAngle = slipAngle + 5*myConstants.deg2rad;
% end
% grid on