function waypointContolForPathPLanning()
% Based on
% algorithm1, @article{SolteroEtAlIJRR14VoronoiPathPlanning,
%   author = {D. E. Soltero and M. Schwager and D. Rus},
%   title = {Decentralized path planning for coverage tasks using gradient descent adaptive control},
%   journal = {International Journal of Robotics Research},
%   month = {March},
%   year = {2014},
%   volume = {33},
%   number = {3},
%   pages = {401--425}}
%
% Srikanth and Aaron T. Becker
% Date 2/11/2015

% generate waypoints and a path

n = 10;
theta = linspace(0,2*pi*(1-1/n),n)';
waypoints = [cos(theta),sin(theta)];
numIters = 5;
f = figure(1);
set(f,'Name','IC path controller')
%make out interesting function
szInteresting = 200;
phi = zeros(szInteresting,szInteresting);
%these are interesting!!!
phi(1:50,1:50) = 1; 
[mX,mY] = meshgrid(linspace(-1,1,szInteresting),linspace(-1,1,szInteresting)); %making the grid

cellsz = mY(2) - mY(1);


plotWaypoints(waypoints,mX,mY,phi)


for iter = 1:2000 % here for these iterations we calculate the previous and next neighbour for each waypoint
    H=[waypoints(:,1),waypoints(:,2)];
    pim=[waypoints(end,:);waypoints(1:end-1,:)];
    pip=[waypoints(2:end,:);waypoints(1,:)];

% function newWayPoints =  moveWaypoints(waypoints)   % algorithm 1
nwaypts = size(waypoints,1);
M_vals = zeros(size(waypoints,1),1); %mass
L_vals = zeros(size(waypoints,1),2); %first mass moment
C_vals = zeros(size(waypoints,1),2); %centroids
% do a for loop over every entry in phi and assign this value to the mass of the correct waypoint.
indExciting = find(phi>0); %index of every 'exciting point'
for m = 1:numel(indExciting)  % iterate through every grid point that is 'exciting'
    indx = indExciting(m);
    pos = [mX(indx)+cellsz/2,mY(indx)+cellsz/2];  %center of grid cell is [mX,mY] + 1/2[cellsize,cellsize], pos is the center of the interesting gridcell
    %squared distance between this grid cell and every waypoint
    sumSqDist = sum((repmat(pos,nwaypts,1) - waypoints).^2,2);
    [~,minIndx] = min(sumSqDist);

    M_vals(minIndx) = M_vals(minIndx)+phi(indx); %calcuate the mass
    L_vals(minIndx,:) = L_vals(minIndx,:)+pos*phi(indx); %calcuate the mass
end
% end
e_vals = zeros(size(waypoints,1),2);
%calculate errors between ccentroid and desired position
C_vals = L_vals./[M_vals,M_vals];
for i= 1:nwaypts
    %errors
    if isnan(C_vals(i,1)) %do for loop, if isnan(C_vals(i)), set error to zero
        e_vals(i,:)=[0,0];
    else
        e_vals(i,:)= C_vals(i,:) - H(i,:);
    end
end
Wn=1;
alpha_vals=zeros(size(waypoints,1),2);
beta_vals=zeros(size(waypoints,1),1);
uir =zeros(size(waypoints,1),2);
for i = 1:nwaypts
        alpha_vals(i,:)=Wn*(pim(i,:)+pip(i,:)-2*H(i,:));
        beta_vals(i,1)=M_vals(i,:)+(2*Wn);
end
% moving towards the centeroid of the voronoi cells
Ki=1; %potentially -time varyin g positive definite matrix
for i = 1:nwaypts
    uir(i,:)=Ki.*((M_vals(i,:).*e_vals(i,:))+alpha_vals(i,:))/beta_vals(i,1); %Control input based on gradient descent
end
%apply control input
deltat=0.1; %time period
waypoints=waypoints+uir*deltat; %updating waypoints 
plotWaypoints(waypoints,mX,mY,phi) % calling function
title(num2str(iter))
pause(.01)
end

    function plotWaypoints(waypoints,mX,mY,phi)
        clf
        pcolor(mX,mY,phi)
        colormap([1,1,1;0,1,0])
        hold on
        hWaypoints = plot(waypoints(:,1),waypoints(:,2),'bo');
        axis equal
        
        hPath = line([waypoints(:,1);waypoints(1,1)],[waypoints(:,2);waypoints(1,2)],'color','m');
        %compute the waypoint's Voronoi Partition
        voronoi(waypoints(:,1),waypoints(:,2));
        
        
    end

end
