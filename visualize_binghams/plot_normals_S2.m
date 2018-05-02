% normals should be 3 by N
function plot_normals(normals, num_normals_to_plot)

hold on;
if nargin>1
    rand_indices = randperm(size(normals,2))';
    indices_to_sample = rand_indices(1:num_normals_to_plot);
    normals_to_plot = normals(:, indices_to_sample);
else
    normals_to_plot = normals;
end
    
for idx=1:size(normals_to_plot, 2)
    normals_to_plot(:,idx) = normals_to_plot(:,idx) / norm(normals_to_plot(:,idx));
end

plot3(normals_to_plot(1,:), normals_to_plot(2,:), normals_to_plot(3,:), '.', 'MarkerSize', 10, 'Color', 'r'); 
xlabel('x');
ylabel('y');
zlabel('z');

[SX,SY,SZ] = sphere(100);
surf(SX,SY,SZ,  'EdgeColor', 'none', 'FaceAlpha', 1);
colormap(gca, gray);

axis vis3d;
axis equal;
hold off;
end