close all; clear; clc;
normals = dlmread('/home/suddhu/Documents/courses/16833/project/staircase_normals/normals/0.txt');
normals = normals';

% normals = [zeros(1,size(normals, 2)) ; normals];
normals = [normals;zeros(1,size(normals, 2))];

% hyperparams
% total normals - 3x248383 
% 0.01 - 2483
is_plot_eigen_vector = 0;
num_normals_to_plot = round(0.1*size(normals,2));
num_normals_to_fit = round(0.5*size(normals,2));
min_point_bmm_fit = round(0.1*num_normals_to_fit);

rand_indices = randperm(size(normals,2))';
indices_to_sample = rand_indices(1:num_normals_to_fit);
normals_to_fit_bmm = normals(:, indices_to_sample);

% [bmm, bmm_weights] = BinghamDistribution.cluster_fit(normals);
[bmm, bmm_weights] = BinghamDistribution.cluster_fit(normals_to_fit_bmm, min_point_bmm_fit);

n_bings = length(bmm);

el = 30;
az = 0;
az_delta = 10;

for idx=1:n_bings+1
    subplot(2,3,idx);
    if idx < n_bings+1
        plot_bingham_S2(bmm(idx), is_plot_eigen_vector, bmm_weights(idx))
    else
        % plot_normals_S2(normals, num_normals_to_plot);
        plot_normals_with_bmm_mode_S2(normals, num_normals_to_plot, bmm, bmm_weights)
    end
end

while 1
    az = az + deg2rad(az_delta);
    for i= 1:n_bings+1
        subplot(2,3,i); 
        view([az,el]);
    end
    pause(0.001);
end

