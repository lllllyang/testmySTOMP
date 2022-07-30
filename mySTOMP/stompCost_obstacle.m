function cost = stompCost_obstacle(robot,radius,Env,vel)

e = 5;
cost = 0;
robot = round(robot);

idx = round(robot/10) + [50, 100, 10];
% idx = round(robot/10) + [50, 60, 30];
%     idx
%     Env(sub2ind([120, 100, 90], idx(:, 2), idx(:, 1), idx(:, 3)))
try
    cost = max(e + radius - Env(sub2ind([300, 250, 130], idx(:, 2), idx(:, 1), idx(:, 3))),0) .* vel;
%     cost = max(e + radius - Env(sub2ind([120, 100, 90], idx(:, 2), idx(:, 1), idx(:, 3))),0) .* vel;
    
    cost = sum(sum(cost));
%     cost = cost / 100;
catch min(min(idx))
    disp('???');
    disp(idx);
end
% for i = 1 : length(robot)
%     idx = round(robot(i,:)/10) + [10, 60, 10];
% %     disp(robot(i,:));
% %     disp(idx);
%     if (idx(1)<0) || (idx(2)<0) || (idx(3)<0)
%         disp(idx);
%     end
%     cost = cost + max( e + radius(i) - Env(idx(2), idx(1), idx(3)),0);
% end 

end