
% output: collision -> binary number that denotes whether this
%                      configuration of the robot is in collision or not. %
function collision = lineCollision(rob,q1,q2,sphereCenter,r)
    stepSize = (q2-q1)/100; % dividing the line joining q1 q2 into 10 parts
    q = q1;
    collision = 0; 
    % initially assume there is no collision and 
    % loop until a collision is encountered or 
    % line is completely checked (whichever comes first)
    for i = 1:1:10
        q = q+stepSize;
        collision = robotCollision(rob,q,sphereCenter,r);
        if(collision==1)
            break;
        end;
    end;
end

