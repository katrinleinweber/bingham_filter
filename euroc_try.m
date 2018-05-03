clear all; 
clc; 
close all; 

% for EUROC data 
E = importdata('/home/suddhu/software/libDirectional_bingham/MH_03_easy_dq-1.txt');
j = 1;
for i = 1:5: length(E) - 4
    D(j,1:4) = E(i+1:i+4); 
    j = j + 1; 
end

for i = 1:size(D,1)-1 
    q_delta(i,:) = get_delta_quat(D(i,:)',D(i+1,:)'); 
    q_delta(i,:) = q_delta(i,:)/norm(q_delta(i,:));
end

% intialization
filter = BinghamFilter();
Z = [-1 -1 -1 0]';

M = [0 0 0 1; 
     0 0 1 0; 
     0 1 0 0; 
     1 0 0 0 ];
% M = eye(4); 
%  B = BinghamDistribution(Z,M);
B = BinghamDistribution(Z,M(:, [1 2 3 4]));

% system noise 
Z_sys = [-5000 -5000 -5000 0]';

M_sys = [0 0 0 1; 
             0 0 1 0; 
             0 1 0 0; 
             1 0 0 0 ];
% M_sys = eye(4); 
B_sys = BinghamDistribution(Z_sys, M_sys(:,[1 2 3 4]));

% measurement noise 
Z_measure = [-500 -500 -500 0]';
M_measure = [0 0 0 1; 
             0 0 1 0; 
             0 1 0 0; 
             1 0 0 0 ];
B_measure = BinghamDistribution(Z_measure, M_measure(:,[1 2 3 4]));

filter.setState(B_sys); 

q_delta_noise = zeros(size(q_delta)); 
% generate noisy data 
for i = 1:size(D,1) - 1    
    B_sample = filter.getEstimate(); 
    A = B_sample.sample(1); 
    z  = quaternionMultiplication(A,q_delta(i,:)');  
    q_delta_noise(i,:) = z; 
end

% init
z_noisy = B.mode(); 
z_clean = B.mode(); 
z_gt = B.mode(); 

filter.setState(B);

% run filter 
for i = 1:size(D,1) -1
    z_noisy = B.mode(); 
    z_clean = B.mode(); 
    
    filter.setState(B);

    filter.updateIdentity(B_sys, q_delta_noise(i,:)'); 
    
    B = filter.getEstimate();
    filter.predictNonlinear(@(x) x, B_sys);
    
    B_predict = filter.getEstimate(); 
   
    z_noisy  = quaternionMultiplication(z_noisy,q_delta_noise(i,:)');     
    z_clean  = quaternionMultiplication(z_clean,B_predict.mode());     
    z_gt = quaternionMultiplication(z_gt,q_delta(i,:)');  
   
    plot_graphs(z_noisy,z_clean); 
    
    %plot_graphs(z_gt,z_clean); 
    
%     100*(i/size(D,1))
end

function plot_graphs(z_noisy,z_clean)
    plot_pause_time = 0.0001;

    subplot(2,2,1); 
    daspect([1 1 1])
    axis vis3d;
    alx1 = plot_quaternions_animate(z_noisy,plot_pause_time*0.1);

    subplot(2,2,2); 
    daspect([1 1 1])
    axis vis3d;
    alx2 = plot_quaternions_animate(z_clean, plot_pause_time); 
   
    orange = [255,165,0]/255; 
    
    subplot(2,2,3);
    plot3( alx1(1,1), alx1(2,1), alx1(3,1), 'r.-' );
    plot3( alx1(1,2), alx1(2,2), alx1(3,2), 'b.-');
    plot3( alx1(1,3), alx1(2,3), alx1(3,3),'.-','Color',orange  );

    xlim([-1 1]); ylim([-1 1]); zlim([-1 1]); 
    hold on; 
    
    subplot(2,2,4); 
    plot3( alx2(1,1), alx2(2,1), alx2(3,1), 'r.-' );
    plot3( alx2(1,2), alx2(2,2), alx2(3,2), 'b.-');
    plot3( alx2(1,3), alx2(2,3), alx2(3,3),'.-', 'Color',orange );
    xlim([-1 1]); ylim([-1 1]); zlim([-1 1]); 
    hold on; 
end

% get quaternion difference
function q_delta = get_delta_quat(qa, qb)
    qa_inv = [qa(1) -qa(2:4)'] ;
    qa_inv = qa_inv / norm(qa_inv);
    q_delta = quaternionMultiplication(qa_inv', qb);
end

% get distance angle b/w quaternions
function dist_q = get_distance_angle(qa, qb)
    dist_q = acos(2*sum(qa.*qb)^2 - 1);
end

% get dist simple b/w quaternions
function dist_q = get_distance_simple(qa, qb)
    dist_q = 1- sum(qa.*qb);
end

