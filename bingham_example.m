clear all; 
clc; 
close all; 
D = importdata('/home/suddhu/Documents/courses/16833/project/quat_0.001_0.3_0.03.csv'); 

E = importdata('/home/suddhu/software/libDirectional_bingham/MH_03_easy_dq-1.txt');

% j = 1;
% for i = 1:5: length(E) - 4
%     D(j,1:4) = E(i+1:i+4); 
%     j = j + 1; 
% end

% subplot(1,2,1); 
% hold on;
% subplot(1,2,2); 
% hold on;

%D = [[1 0 0 0]; D];
%D = D(2:1000,:);
plot_pause_time = 0.01;

for i = 1:size(D,1)-1 
    q_delta(i,:) = get_delta_quat(D(i,:)',D(i+1,:)'); 
end

filter = BinghamFilter();
Z = [-200 -200 -200 0]';
M = [0 0 0 1; 
     0 0 1 0; 
     0 1 0 0; 
     1 0 0 0 ];
%  B = BinghamDistribution(Z,M);
B = BinghamDistribution(Z,M(:, [1 2 3 4]));

Z_sensor = [-20000 -20000 -20000 0]';
M_sensor = [0 0 0 1; 
             0 0 1 0; 
             0 1 0 0; 
             1 0 0 0 ];
%M_sensor = [0 0 0 1; 0 0 1 0; 0 1 0 0; 1 0 0 0 ];
B_sensor = BinghamDistribution(Z_sensor, M_sensor(:,[1 2 3 4]));
% B_sensor = BinghamDistribution(Z_sensor, M_sensor(:,[ 3 1 2 4]));
% B_sensor = BinghamDistribution(Z_sensor, M_sensor(:,[ 3 1 2 4]));

%figure('units','normalized','outerposition',[0 0 1 1])
filter.setState(B);    


filter.setState(B);    

%% update identity with different measurement
for i = 1:size(D,1) -1
    B.compose(B_sensor); 
    A = B.sample(1); 
            
    z  = quaternionMultiplication(B.mode(),q_delta(i,:)');     
    z = z/norm(z); 
    filter.predictNonlinear(@(x) z, B_sensor);
    B = filter.getEstimate();
    
    Bingham_predict(:,i) = B.mode(); 
    Z(:,i) = z;
    
    %get_distance_simple(Bingham_predict(:,i)', z')

    subplot(2,2,1); 
    daspect([1 1 1])
    axis vis3d;
    alx1 = plot_quaternions_animate(A,plot_pause_time*0.1);


    subplot(2,2,2); 
    daspect([1 1 1])
    axis vis3d;
    alx2 = plot_quaternions_animate(Bingham_predict(:,i), plot_pause_time); 

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

Bingham_predict = Bingham_predict';


for i = 1:size(Bingham_predict,1)
    dist_q(i,1) = get_distance_simple(Bingham_predict(i,:), D(i+1,:));
end


% for i = 1:size(D,1)
% subplot(1,2,1); 
% plot_quaternions_animate(D(i,:), 0.0001);
% 
% subplot(1,2,2); 
% plot_quaternions_animate(Bingham_predict(i,:), 0.0001); 
% end


function q_delta = get_delta_quat(qa, qb)
    qa_inv = [qa(1) -qa(2:4)'] ;
    qa_inv = qa_inv / norm(qa_inv);
    q_delta = quaternionMultiplication(qa_inv', qb);
%     q_delta = quaternionMultiplication(qInv(qa), qb);
    
%     q_delta = quaternionMultiplication(qb,qa_inv');
end


function dist_q = get_distance_angle(qa, qb)
    dist_q = acos(2*sum(qa.*qb)^2 - 1);
end

function dist_q = get_distance_simple(qa, qb)
    dist_q = 1- sum(qa.*qb);
end

