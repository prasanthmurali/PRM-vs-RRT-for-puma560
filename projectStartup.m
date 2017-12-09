function [isPathPresent,runningTime] = projectStartup(noOfIterations,...
                                                    connectionDist) 
   
    % set up robot and obstacles
    rob = createRobot();
    qStart = [0 -0.78 0 -0.78 0 0];
    xGoal = [-0.3;0.5;0.5];
    qGoal = rob.ikine(transl(xGoal),zeros(1,6),[1,1,1,0,0,0]);
    sphereCenter1 = [-0.75;0.5;0];
    sphereCenter2 = [0.5;0.5;0];
    sphereCenter3 = [0.5;-0.5;0];
    sphereCenter4 = [-0.5;-0.5;-0.75];
    sphereCenter5 = [-0.5;0.5;-0.75];
    sphereCenter6 = [0.5;0.5;-0.75];
    sphereCenter7 = [-0.5;-0.5;0];
    sphereCenter8 = [0.5;-0.5;-0.75];
    sphereCenter9 = [0;0.5;0];
    sphereRadius = [0.2;0.2;0.2;0.2;0.2;0.2;0.2;0.2;0.2];
    sphereCenter = [sphereCenter1';sphereCenter2';sphereCenter3';...
        sphereCenter4';sphereCenter5';sphereCenter6';sphereCenter7';...
        sphereCenter8';sphereCenter9'];
    cylinderCenter1 = [0.5;-0.5;-0.75];
    cylinderCenter2 = [-0.5;-0.5;-0.75];
    cylinderCenter3 = [0.75;0.75;-1];
    cylinderCenter4 = [-0.75;0.75;-1];
    cylinderCenter = [cylinderCenter1';...
        cylinderCenter3';cylinderCenter4'];
    cylinderRadius = [0.2;0.2;0.2];
    cylinderCenter=[cylinderCenter1';cylinderCenter2'];
    cylinderRadius=[0.2;0.2];
    
    % plot robot and obstacles 
    rob.plot(qStart);
    hold on;
    plotObstacles(sphereCenter,sphereRadius,cylinderCenter,cylinderRadius);

    % Call PRM for the given robot and configuration space
    % to get the graph back
    
    % tic;
    [V,E]= PRMap(rob,sphereCenter,sphereRadius,...
        cylinderCenter,cylinderRadius,noOfIterations,connectionDist);
%      [qMilestones] = RapidRandomTree(rob,sphereCenter,sphereRadius,...
%          qStart,xGoal,cylinderCenter,cylinderRadius,noOfIterations);
%      runningTime = toc;
    
%     Query Phase of PRM
    [startNode,path] = findClosest(V,qStart);
    if(not(path==9999))
        [endNode,path] = findClosest(V,qGoal);
        G = graph(E);
        if(not(path==9999))
            path = shortestpath(G,startNode,endNode);
            pathSize = size(path,2);
        end
    else
        isPathPresent=0;
    end
    if(pathSize>1)
        isPathPresent=1;
    else
        isPathPresent=0;
    end
    
%    plotting the collision free path
    qMilestones = updatePath(V,path,qStart,qGoal);
    
%   RRT Experiments
%     pathSize=size(qMilestones,1);
%     if(pathSize>2)
%         isPathPresent = 1;
%     else
%         isPathPresent = 0;
%     end
%     
    qTraj = interpMilestones(qMilestones);
    rob.plot(qTraj);
    
end

% GIVEN: The centers and radii of obstacles
% RETURNS: Plots the obstales on the configuration space
function plotObstacles(sphereCenter, sphereRadius, cylinderCenter,...
    cylinderRadius)
    noOfSpheres = size(sphereRadius,1);
    for i =1:1:noOfSpheres
        hold on;
        drawSphere(sphereCenter(i,:),sphereRadius(i));
    end
    noOfCylinders = size(cylinderRadius,1);
    for i =1:1:noOfCylinders
        hold on;
        drawCylinder(cylinderCenter(i,:),cylinderRadius(i));
    end
end

% GIVEN: Graph V, path from start to end node, initial and final 
%        configurations of the robot
% RETURNS: A vector of configurations the robot needs to take to move 
%          from start to goal position
function qMilestones = updatePath(V,path,qStart,qGoal)
    qMilestones = qStart;
    pathLength = size(path,2);
    for i=1:1:pathLength
        qMilestones=[qMilestones;V(path(i),:)];
    end
    qMilestones=[qMilestones;qGoal];
end

% GIVEN: The matrix denoting the joint angles for the path of robot
% RETURNS: The matrix denoting the trajectory (for plotting)
function traj = interpMilestones(qMilestones)
    d = 0.05;
    traj = [];
    for i=2:size(qMilestones,1)
        delta = qMilestones(i,:) - qMilestones(i-1,:);
        m = max(floor(norm(delta) / d),1);
        vec = linspace(0,1,m);
        leg = repmat(delta',1,m) .* repmat(vec,size(delta,2),1) + ...
            repmat(qMilestones(i-1,:)',1,m);
        traj = [traj;leg'];   
    end
end

% GIVEN: no parameter
% RETURNS: A robot with the desired configuration (puma_560 in this case)
function rob = createRobot()
    mdl_puma560;
    rob = p560;
end

%GIVEN: The center and diameter of a sphere
%RETURNS: Plots that sphere
function drawSphere(position,diameter)
    [X,Y,Z] = sphere;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    surf(X,Y,Z);
end

%GIVEN: The center and diameter of a cylinder
%RETURNS: Plots that cylinder
function drawCylinder(position,diameter)
    [X,Y,Z] = cylinder;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    Z(2, :) = 0.05;
    surf(X,Y,Z);
end

function [node,path] = findClosest(V,qRandom)
    minDist = inf;
    node = 9999;
    for i = 1:1:size(V,1)
        qTemp = V(i,:);
        eucDist = (qRandom-qTemp);
        eucDist = norm(eucDist);
        if(eucDist<minDist)
            minDist = eucDist;
            node = i;
            q = qTemp;
        end
    end
    if(node==9999)
        path=9999;
        node=9999;
    else
        path=1;
    end
end