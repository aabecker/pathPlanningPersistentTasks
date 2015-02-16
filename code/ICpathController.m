function ICpathController()
% Based on
% algorithm1, @article{SolteroEtAlIJRR14VoronoiPathPlanning,
%   author = {D. E. Soltero and M. Schwager and D. Rus},
%   title = {Decentralized path planning for coverage tasks using gradient descent adaptive control},
%   journal = {International Journal of Robotics Research},
%   month = {March},
%   year = {2014},
%   volume = {33},
%   number = {3},
%   pages = {401--425}
% }
%
% Srikanth and Aaron T. Becker
%
%  TODO: 
% 0. fix the enpoint point i-1, i+1 problems
% 1. draw the centroids of the 'interesting regions' inside each
%  waypoint's voronoi cell.
% 
% Date 2/11/2015

% generate waypoints and a path
n = 10;
theta = linspace(0,2*pi*(1-1/n),n)';
waypoints = [cos(theta),sin(theta)];
numIters = 5;
f = figure(1);
set(f,'Name','IC path controller')
%make out interesting function
szInteresting = 20;
phi = zeros(szInteresting,szInteresting);
phi(1:5,1:5) = 1;  %these are interesting!
[mX,mY] = meshgrid(linspace(-1,1,szInteresting),linspace(-1,1,szInteresting));
pcolor(mX,mY,phi)
colormap([1,1,1;0,1,0])
hold on
cellsz = mY(2) - mY(1); % the length of a grid cell



hWaypoints = plot(waypoints(:,1),waypoints(:,2),'bo');
axis equal

hPath = line([waypoints(:,1);waypoints(1,1)],[waypoints(:,2);waypoints(1,2)],'color','m');

for j = 1:numIters
 %plot the interesting function phi
 waypoints =  moveWaypoints(waypoints);
 % draw the waypoints
    set(hWaypoints,'xdata',waypoints(:,1),'ydata',waypoints(:,2));
    axis equal
    set(hPath,'xdata', [waypoints(:,1);waypoints(1,1)],'ydata',[waypoints(:,2);waypoints(1,2)]);
 
 
end
 
 


    function newWayPoints =  moveWaypoints(waypoints)
        % algorithm 1
        
        for i = 1:size(waypoints,1)
            %compute the waypoint's Voronoi Partition
            voronoi(waypoints(:,1),waypoints(:,2));  %just for visualization
            %Note   For the topology of the Voronoi diagram, i.e., the vertices for each Voronoi cell, use voronoin.
  
              
            % compute C_i according to 3
            C_vals = zeros(size(waypoints(:,1)));
            % do a for loop over every entry in phi and assign this value to the mass of the correct waypoint.
            indExctiting = find(phi>0); %index of every 'exciting point'
            for m = 1:numel(indExctiting)  % iterate through every grid point that is 'exciting'
                indx = indExctiting(m);
                pos = [mX(indx)+cellsz/2,mY(indx)+cellsz/2];  %center of grid cell is [mX,mY] + 1/2[cellsize,cellsize]
                %squared distance between this grid cell and every waypoint
                
                sumSqDist = sum((repmat(pos, size(waypoints,1),1) - waypoints).^2,2);
                [~,minIndx] = min(sumSqDist);
                
                C_vals(minIndx) = C_vals(minIndx)+phi(indx);
                % TODO: also calculate L_i
                
                %TODO: also valculate the moment?
            end

            
            
            % Obtain neighbor weighpoint locations p
            pim1 = waypoints(i-1,:);  %todo: add if statements if i = 1 or i = n
            pip1 = waypoints(i+1,:);
            
            %compute u_i^r according to (5)
            uir = 1; %TODO
            
            %update p_i according to (5)
            waypoints(i,:) = waypoints(i,:) + Kmove*uir;
            
            
        end
        
    end



end




