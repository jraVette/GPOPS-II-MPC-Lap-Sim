function generateTrackFigure(varargin)
%Function this function will plot a track with distance waylines

defaults  = {'trackName'  'Nurburgring.mat'
            'flagBuffer' 10
             'waylines' [250:250:4500] 
             'plotArrow' 'true'
             'offsetTextX' 10
             'offsetTextY' 30
             'plotBuffer'  1.2
             'flagLength' 150};         
                  

% defaults  = {'trackName'  'Hockenheim.mat'
%             'flagBuffer' 6
%              'waylines' [100:100:2640] 
%              'plotArrow' 'true'
%              'offsetTextX' 5
%              'offsetTextY' 15
%              'plotBuffer'  1.2
%              'flagLength' 70};         
%                   
%          
% defaults  = {'trackName'  'PathInfoChicane.mat'
%              'flagBuffer' 2
%              'waylines' [50:50:649.5] 
%              'plotArrow' 'true'
%              'offsetTextX' 5
%              'offsetTextY' 15
%              'plotBuffer'  1.2
%              'flagLength' 30};         
                  
         
setDefaultsForVarargin(defaults,varargin)


load(trackName)
track = frenetToXYFrameTrajectory(track.curvature.meas,track.distance.meas,zeros(size(track.distance.meas)),zeros(size(track.distance.meas)),0,0,0,10);
delete(findall(gcf,'Type','line','Color','b'))
set(findall(gcf,'Type','line','Color','k'),'linewidth',2.0)
legend off

x0 = 0;
y0 = 0;
psi0 = 0;

rawTrackData = frenetToXYFrameTrajectory(track.curvature.meas,...
                                         track.distance.meas,...
                                         zeros(size(track.distance.meas)),...
                                         zeros(size(track.distance.meas)),...
                                         x0,y0,psi0,...
                                         track.trackWidth.meas(1),...
                                         'suppressPlot',true);
                                     
vars = {'distance' 'distance'
        'xRoadUp' 'xRoadUp'
        'xRoadLow' 'xRoadLow'
        'yRoadUp' 'yRoadUp'
        'yRoadLow' 'yRoadLow'
        'xPath'   'xPath'
        'yPath'   'yPath'
        'trackWidth' 'trackWidth'};
    tempDaq.rawData = rawTrackData;
    getChannelDataFromDaqFile(tempDaq,vars);
                                         
if plotArrow

            
    roadWidthFlag = flagBuffer*norm([xRoadUp(1), yRoadUp(1)]-[xRoadLow(1), yRoadLow(1)]);  %0.45;
%     lapDistance = distance(1) - distance(end);                  %to make teh arrow a fraciton of the lap distance
%     flagLength = 50; %[m] 0.1*lapDistance
    flagYBuffer = 10; %[m] to control jut out past track
    
    yFlagLower = y0-roadWidthFlag/2 - trackWidth(1);
    yFlagUpper = y0+roadWidthFlag/2 + trackWidth(1);
    
    line([x0 x0],[yFlagLower yFlagUpper],'linestyle','-','Color','k','linewidth',1.5)
    arrow([x0,yFlagLower],[x0+flagLength,yFlagLower],'linewidth',1.5)
    arrow([x0,yFlagUpper],[x0+flagLength,yFlagUpper],'linewidth',1.5)
    
end


localTrack = trackParametersAtLoopedDistance(track,waylines);

for i = 1:length(waylines)
    x = localTrack.xCar.meas(i);
    y = localTrack.yCar.meas(i);
    line(x,y,'marker','s','markersize',10,'linewidth',1.5,'markerfacecolor',0.3*ones(3,1))
    dx = offsetTextX;
    dy = offsetTextY;
    text(x+dx,y+dy,sprintf('%4.1fm',waylines(i)))
end

xlim(findNewLimitsWithBuffer(xPath,plotBuffer))
ylim(findNewLimitsWithBuffer(yPath,plotBuffer))

xlabel('X [m]'); ylabel('Y [m]')
publicationStyleAxes
[~,justFilename,~] = fileparts(trackName);
filename = justFilename;
title(filename)
saveas(gcf,filename)
print(gcf,'-depsc2',[filename '.eps'])