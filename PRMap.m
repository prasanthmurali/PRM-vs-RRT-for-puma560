% Returns the graph after the pre-processing phase 
% by applying the Probabilistic Roadmap algorithm
function [V,E] = PRMap(rob,sphereCenter,sphereRadius,...
    cylinderCenter,cylinderRadius,noOfIterations,connectionDist)
 connectDist = connectionDist;
 V=[];
 E = zeros(noOfIterations,noOfIterations);
 for i=1:1:noOfIterations
     qRandom = -pi + 2*pi*rand(6,1);
     qRandom = qRandom';
     if(not(isRobotColliding(rob,qRandom',sphereCenter,sphereRadius,...
             cylinderCenter,cylinderRadius)))  
         E = createGraph(rob,V,E,qRandom,sphereCenter,sphereRadius,...
             cylinderCenter,cylinderRadius,connectDist);
         V = [V; qRandom];      
     end
 end
end

% Returns 1(true) if the robot is in collision with any of the obstacles,
% false otherwise
function isCollision = isRobotColliding(rob,qRandom,sphereCenter,...
    sphereRadius,cylinderCenter,cylinderRadius)
    noOfObstacles = size(sphereCenter,1);
    isCollision = 0;
    for i=1:1:noOfObstacles
        if(robotCollision(rob,qRandom,sphereCenter(i,:)',sphereRadius(i)))
            isCollision=1;
            return;
        end
    end
    noOfObstacles = size(cylinderCenter,1);
    isCollision = 0;
    for i=1:1:noOfObstacles
        if(checkForCylinderCollision(rob,qRandom,cylinderCenter(i,:)',...
        cylinderRadius(i)))
            isCollision=1;
            return;
        end
    end 
end

% Returns true if the robot is in collision with cylindrical obstacles,
% false otherwise
function isCollision = checkForCylinderCollision(rob,qRandom,...
    cylinderCenter,cylinderRadius)
    q=qRandom;
    x1 = [0;0;0];
    T2 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q);
    x2 = T2(1:3,4);
    T3 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q);
    x3 = T3(1:3,4);
    T4 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q) * rob.A(5,q);
    x4 = T4(1:3,4);
    T5 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q) * rob.A(5,q)*...
        rob.A(6,q);
    x5 = T5(1:3,4);    
    x =[x1';x2';x3';x4';x5'];
    isCollision = isInsideCylinder(x,cylinderCenter,cylinderRadius);
end

% Returns true if a point in cartesian space is inside the cylinder
function isInside=isInsideCylinder(x,cylinderCenter,cylinderRadius)
    noOfJoints = size(x,1);
    isInside = 0;
    height = 2;
    for i=1:1:noOfJoints
        jointPoint =x(i,:);
        pointDist = jointPoint - cylinderCenter;
        pointDistsq = pointDist * pointDist';
        if(pointDistsq<cylinderRadius*cylinderRadius)
            cylinderTop = cylinderCenter(3)+height;
            cylinderBottom = cylinderCenter;
            if(jointPoint(3)<cylinderTop & jointPoint(3)>cylinderBottom)
                isInside = 1;
                return;
            end
        end
    end
end

% Connects the edges to all nearby nodes of a particular node in a graph
function E = createGraph(rob,V,E,qRandom,sphereCenter,sphereRadius,...
    cylinderCenter,cylinderRadius,connectDist)
    neighboursMap = getAllNearestNeighbours(rob,V,qRandom,sphereCenter,...
        sphereRadius,cylinderCenter,cylinderRadius,connectDist);
    n = size(V,1) + 1;
    l = size(neighboursMap);
    for i=1:1:l
        E(i,n)=1;
        E(n,i)=1;
    end
end

% Returns a graph depicting all free nodes
function neighboursMap = getAllNearestNeighbours(rob,V,qRandom,...
    sphereCenter,sphereRadius,cylinderCenter,cylinderRadius,connectDist)
    neighboursMap=[];
    n = size(V,1);
    for i=1:1:n
        qTemp = V(i,:);
        if(not(isPathCollision(rob,qTemp',qRandom',sphereCenter,...
                sphereRadius,cylinderCenter,cylinderRadius)))
            eucDist = (qRandom-qTemp);
            eucDist = norm(eucDist);
            if(eucDist<connectDist)   
                neighboursMap = [neighboursMap;i];
            end
        end
    end
end

% Returns true if the line joining two nodes is in collision with any of
% the obstacles
function isCollision = isPathCollision(rob,qTemp,qRandom,sphereCenter,...
    sphereRadius,cylinderCenter,cylinderRadius)
    noOfObstacles = size(sphereCenter,1);
    isCollision = 0;
    for i=1:1:noOfObstacles
        if(lineCollision(rob,qTemp,qRandom,sphereCenter(i,:)',sphereRadius(i)))
            isCollision=1;
            return;
        end
    end
    noOfObstacles = size(cylinderCenter,1);
    isCollision = 0;
    for i=1:1:noOfObstacles
        if(checkForPathCollisionWithCylinder(rob,qTemp,qRandom,...
        cylinderCenter(i,:)',cylinderRadius(i)))
            isCollision=1;
            return;
        end
    end
end

% Returns true if the line joining two nodes is in collision with the
% cylindrical obstacles
function isCollision = checkForPathCollisionWithCylinder(rob,qTemp,...
    qRandom,cylinderCenter,cylinderRadius)
    q1=qTemp;
    q2=qRandom;
    stepSize = (q2-q1)/100;
    q = q1;
    isCollision = 0; 
    for i = 1:1:10
        q = q+stepSize;
        isCollision = checkForCylinderCollision(rob,q,cylinderCenter,...
        cylinderRadius);
        if(isCollision==1)
            break;
        end
    end
end