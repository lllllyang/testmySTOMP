function [Env,all_cubes] = constructEnv(voxel_size, myspace, cartis_obs)
% Environment construction here
% Constructing binary map for discretized 3D voxels in workspace
% For simplicity, no API provided, user directly edit this file

%%
%Env Init
% voxel_size = [10, 10, 10];
Env_size = [-500, -1000, -100; 2000, 2000, 1200]; %Front-top-left point for 1st row, length-wigth-height for second
% Env_size = [0, 0, 0; 1500, 1800, 1200]; %Front-top-left point for 1st row, length-wigth-height for second
% please change the paramters in the stompCost_obstacle

%All free space
Env = zeros((Env_size(2, 2)-Env_size(1, 2)) / voxel_size(2), (Env_size(2, 1)-Env_size(1, 1)) / voxel_size(1), (Env_size(2, 3)-Env_size(1, 3)) / voxel_size(3));
%XYZ representation for each voxel
% [X, Y, Z] = meshgrid(Env_size(1, 1) + 0.5 * voxel_size(1) : voxel_size(1) : Env_size(1, 1) + Env_size(2, 1) - 0.5 * voxel_size(1), ...
%     Env_size(1, 2) + 0.5 * voxel_size(2) : voxel_size(2) : Env_size(1, 2) + Env_size(2, 2) - 0.5 * voxel_size(2), ...
%     Env_size(1, 3) + 0.5 * voxel_size(3) : voxel_size(3) : Env_size(1, 3) + Env_size(2, 3) - 0.5 * voxel_size(3));

%%
%Follow this if you want to add a cube
% cube = [150, 0, 100;275, 225, 100]; % Follows the env_size
% cube_2 = cube;
% cube = [floor(cube(1, :)./voxel_size); ceil((cube(1, :) + cube(2, :))./voxel_size)] + -Env_size(1, :) ./ voxel_size;
% [x, y, z] = meshgrid(cube(1, 2):cube(2, 2), cube(1, 1):cube(2, 1), cube(1, 3):cube(2, 3));
% Env(sub2ind([Env_size(2, 2) / voxel_size(2), Env_size(2, 1) / voxel_size(1), Env_size(2, 3) / voxel_size(3)], x, y, z)) = 1;
all_cubes=[];
% Follow this if you want to add a cube
for each_o=1:length(cartis_obs)
    
    each_obs=cartis_obs(each_o)
    cubic0=myspace{each_obs,3}-myspace{each_obs,2}/2;
    cubic1=myspace{each_obs,2};
%     cubic1=myspace{each_obs,3}+myspace{each_obs,2}/2;
    cube0=[cubic0; cubic1;];
%     cube=[cube0(:,2) cube0(:,1) cube0(:,3)];
    cube=cube0*1000;

all_cubes=[all_cubes; cube;];
    
cube = [floor(cube(1, :)./voxel_size); ceil((cube(1, :) + cube(2, :))./voxel_size)] + -Env_size(1, :) ./ voxel_size;
[x, y, z] = meshgrid(cube(1, 2):cube(2, 2), cube(1, 1):cube(2, 1), cube(1, 3):cube(2, 3));
% Env(sub2ind([Env_size(2, 2) / voxel_size(2), Env_size(2, 1) / voxel_size(1), Env_size(2, 3) / voxel_size(3)], x, y, z)) = 1;
Env(sub2ind([(Env_size(2, 2)-Env_size(1, 2)) / voxel_size(2), (Env_size(2, 1)-Env_size(1, 1)) / voxel_size(1), (Env_size(2, 3)-Env_size(1, 3)) / voxel_size(3)], x, y, z)) = 1;
end

% disp('where is the obs?')
% %%
% %Follow this if you want to add a sphere/ellipse
% sphere = [140, 180, 280;1, 1, 1];
% r = 60;
% k = (sphere(2, 1) * (X - sphere(1, 1))).^ 2 + (sphere(2, 2) * (Y - sphere(1, 2))).^2 + (sphere(2, 3) * (Z - sphere(1, 3))).^2 < (r ^ 2);
% Env(k == 1) = 1;
% 
% %%
% %Follow this if you want to add a sphere/ellipse
% sphere = [220, 100, 200;1, 1, 1];
% r = 60;
% k = (sphere(2, 1) * (X - sphere(1, 1))).^ 2 + (sphere(2, 2) * (Y - sphere(1, 2))).^2 + (sphere(2, 3) * (Z - sphere(1, 3))).^2 < (r ^ 2);
% Env(k == 1) = 1;

%%
%Add arbitary obstacles here
%Subtract a small tube from a large one
% cube = [100, 200, 100;115, 160, 180]; % Follows the env_size
% cube = [floor(cube(1, :)./voxel_size); ceil((cube(1, :) + cube(2, :))./voxel_size)] + -Env_size(1, :) ./ voxel_size;
% [x, y, z] = meshgrid(cube(1, 2):cube(2, 2), cube(1, 1):cube(2, 1), cube(1, 3):cube(2, 3));
% Env(sub2ind([Env_size(2, 2) / voxel_size(2), Env_size(2, 1) / voxel_size(1), Env_size(2, 3) / voxel_size(3)], x, y, z)) = 1;
% cube = [100, 200, 150;115, 80, 80]; % Follows the env_size
% cube = [floor(cube(1, :)./voxel_size); ceil((cube(1, :) + cube(2, :))./voxel_size)] + -Env_size(1, :) ./ voxel_size;
% [x, y, z] = meshgrid(cube(1, 2):cube(2, 2), cube(1, 1):cube(2, 1), cube(1, 3):cube(2, 3));
% Env(sub2ind([Env_size(2, 2) / voxel_size(2), Env_size(2, 1) / voxel_size(1), Env_size(2, 3) / voxel_size(3)], x, y, z)) = 0;
% 
% %Subtract a small half-sphere from a large one
% cube = [100, -100, 100;80, 80, 200]; % Follows the env_size
% cube = [floor(cube(1, :)./voxel_size); ceil((cube(1, :) + cube(2, :))./voxel_size)] + -Env_size(1, :) ./ voxel_size;
% [x, y, z] = meshgrid(cube(1, 2):cube(2, 2), cube(1, 1):cube(2, 1), cube(1, 3):cube(2, 3));
% Env(sub2ind([Env_size(2, 2) / voxel_size(2), Env_size(2, 1) / voxel_size(1), Env_size(2, 3) / voxel_size(3)], x, y, z)) = 1;
% sphere = [140, -70, 200;2, 2, 1];
% r = 60;
% k = (sphere(2, 1) * (X - sphere(1, 1))).^ 2 + (sphere(2, 2) * (Y - sphere(1, 2))).^2 + (sphere(2, 3) * (Z - sphere(1, 3))).^2 < (r ^ 2);
% Env(k == 1) = 0;

%You can always stack per-layer obstacle to form a 3D non-convex obstacle
end