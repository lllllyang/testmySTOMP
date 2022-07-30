
[a,b,c]=size(Env)
figure;
for z = 1:c
    for h = 1:a % y
        for l = 1:b %x
            if Env(h,l,z) == 1
                plot3(h,l,z,'bo'); hold on;
            end
        end
    end
end
xlim([0 100+50]);
ylim([0 120+60]);
zlim([0 90+30]);
xlabel('x')
ylabel('y')
zlabel('z')
view(80,20)
e=5
% for i= 1: length(theta)
%     [X,~]=updateQ(theta(:,i)');    
% [robotF, robotR] = stompRobot_Formation(X);
%     if i == 1
%         robotFO = robotF;
%     end
% idx = round(robotF/10) + [50, 60, 0];
% vel = sqrt(sum((robotF - robotFO).^2, 2));
% radius=robotR;
%  cost = max(e + radius - Env(sub2ind([180, 150, 120], idx(:, 2), idx(:, 1), idx(:, 3))),0) .* vel;
% plot3(idx(:,1),idx(:,2),idx(:,3)); hold on;
% end


% Env_size = [-500, -600, 0; 1000, 1200, 1200]; %Front-top-left point for 1st row, length-wigth-height for second
% voxel_size = [10, 10, 10];
% [Env,Cube] = constructEnv(voxel_size);
% % cube = [150, -400, 100;275, 225, 200]; % Follows the env_size
% cube = [150,0, 100;275, 225, 100]; % Follows the env_size
% cube = [floor(cube(1, :)./voxel_size); ceil((cube(1, :) + cube(2, :))./voxel_size)] + -Env_size(1, :) ./ voxel_size
% 
% 
% [y, x, z] = meshgrid(cube(1, 2):cube(2, 2), cube(1, 1):cube(2, 1), cube(1, 3):cube(2, 3));
% Env(sub2ind([(Env_size(2, 2)-Env_size(1, 2)) / voxel_size(2), (Env_size(2, 1)-Env_size(1, 1)) / voxel_size(1), Env_size(2, 3) / voxel_size(3)], y, x, z)) = 1;

% sub2ind([Env_size(2, 2) / voxel_size(2), Env_size(2, 1) / voxel_size(1), Env_size(2, 3) / voxel_size(3)], y, x, z)
% [x, y, z] = meshgrid(cube(1, 2):cube(2, 2), cube(1, 1):cube(2, 1), cube(1, 3):cube(2, 3));










% sub2ind([150, 180, 120], idx(:, 2), idx(:, 1), idx(:, 3))
% 
% q1=idx(:, 2);
% q2=idx(:, 1);
% q3=idx(:, 3);
% % figure;
% 
% 
% 
% 
% 
% row = [1 2 3 1];
% col = [2 2 2 3];
% sz = [3 3];
% ind = sub2ind(sz,row,col)

% see=robotF;
% figure;
% for z = 1:c
%     x = see(:,:,z);
%     z
%     [hang,lie]=size(x);
%     for h = 1:hang
%         for l = 1:lie
%             if see(h,l,z) == 1
%                 plot3(h,l,z,'bo'); hold on;
% %             else
% %                 plot3(h,l,z,'ro'); hold on;
%             end
%         end
%     end
% end
% xlim([0 120]);
% ylim([0 100]);
% zlim([0 90]);
% xlabel('x')
% ylabel('y')
% zlabel('z')

% jbx=Env(1,1,1)
% 
% 
% % figure;
% % slice(Env)
% 
% [X,Y,Z] = meshgrid(-2:.2:2);
% V = X.*exp(-X.^2-Y.^2-Z.^2);
% 
% xslice = [-1.2,0.8,2];   
% yslice = [];
% zslice = 0;
% slice(X,Y,Z,V,xslice,yslice,zslice)