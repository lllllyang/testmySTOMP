clear all;close all;

%%
%Parameters
T = 5;
nSamples = 100;
kPaths = 4;
convThr = 0;
% T = 5;
% nSamples = 100;
% kPaths = 20;
% convThr = 0;
%%
%Setup environment
jb=lynxStart();hold on;
%Environment size
Env = zeros(100,100,100);
% %Obstacle cube
% obsts = [100 1000 -1000 1000 200 200];
obsts=[];
% %Passage hole [center r]
% hole = [0 0 200 60];
hole=[];
%Calculate EDT_Env
voxel_size = [10, 10, 10];
[Env,Cube] = constructEnv(voxel_size);
Env_edt = prod(voxel_size) ^ (1/3) * sEDT_3d(Env);
% Env_edt = sEDT_3d(Env);
