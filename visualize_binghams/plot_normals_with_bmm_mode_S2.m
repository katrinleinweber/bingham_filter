function plot_normals_with_bmm_mode_S2(normals, num_normals_to_plot, bmm_array, bmm_weights)

scale_quiver = 1.5;
plot_normals_S2(normals, num_normals_to_plot);
n_bings = length(bmm_array);
hold on;

for idx=1:n_bings
    bing_curr = bmm_array(idx);
    mode_curr = scale_quiver * bing_curr.mode;
    quiver3(0,0,0,mode_curr(1),mode_curr(2),mode_curr(3),'linewidth',3, 'color', 'k');
    quiver3(0,0,0,-mode_curr(1),-mode_curr(2),-mode_curr(3),'linewidth',3, 'color', 'k');

    text(mode_curr(1), mode_curr(2), mode_curr(3),[num2str(bmm_weights(idx))],'HorizontalAlignment','left','FontSize',12);
end

axis vis3d;
axis equal;
hold off;
