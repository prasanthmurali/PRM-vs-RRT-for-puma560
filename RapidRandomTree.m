% output -> qMilestones -> 6xn vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. 
function qMilestones = RapidRandomTree(rob,sphereCenter,sphereRadius,...
    qStart,xGoal,cylinderCenter,cylinderRadius,noOfIterations)

    % Inverse Kinematics to get qGoal from xGoal
    qGoal = rob.ikine(transl(xGoal),zeros(1,6),[1,1,1,0,0,0]);  
    tree = qStart;
    nodeMap = ones(300,1);
    
    for i = 1:1:noOfIterations
        
        %Randomize with goal bias 
        if(mod(i,10)==0)
            qRandom = qGoal;
        else
        qRandom = -pi + 2*pi*rand(6,1);
        qRandom = qRandom';
        end 
        
        % Now, check if the random point is collision free. If this passes,
        % find the closest point on the tree to the random point.
        % Check if the line joining these two points is collision free,
        % and if that passes, add the point to the tree and update the 
        % reference to parent. The cycle breaks when qGoal is encountered
        % as the random point due to goal bias.
         if(not(isRobotColliding(rob,qRandom',sphereCenter,sphereRadius,...
             cylinderCenter,cylinderRadius))) 
            [q,pos] = findClosest(tree,qRandom);
            if(not(isPathCollision(rob,q',qRandom',sphereCenter,...
                sphereRadius,cylinderCenter,cylinderRadius)))
                n = size(tree,1);
                nodeMap(n+1) = pos;
                tree = [tree;qRandom];
                if(isequal(qGoal,qRandom))
                    break;
                end          
            end
        end
    end   
    % Once we have the tree, traverse to find the path connecting goal to
    % start.
    qMilestones = updatePath(tree,nodeMap,n,qStart);
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

% INPUT: The tree, nodeMap, length of tree and initial Configuration
% RETURNS: The path from start to end when traversing through the tree %
function qMilestones = updatePath(tree,nodeMap,n,qStart)
    q = [];
    qMilestones= [];
    pos = inf;
    n = n+1;
    while(pos>1)
        pos = nodeMap(n);
        q = tree(n,:);
        qMilestones = [q;qMilestones];
        if(pos==1)
            qMilestones= [tree(1,:);qMilestones];
        end
        n = pos;
    end
end

% INPUT: The tree and a random configuration
% RETURNS: The closest node in the tree to the random configuration
%          and its position in the tree
function [q,pos] = findClosest(tree,qRandom)
    pos = inf;
    minDist = inf;
    for i = 1:1:size(tree,1)   
        qTemp = tree(i,:);
        eucDist = (qRandom-qTemp); 
        eucDist = norm(eucDist);
        if(eucDist<minDist)
            minDist = eucDist;
            pos = i;
            q = qTemp;
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

% Returns true if the robot is in collision with cylindrical obstacles,
% false otherwise
function isCollision = checkForCylinderCollision(rob,qRandom,cylinderCenter,...
        cylinderRadius)
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