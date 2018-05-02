function plot_bingham_S2(bing, is_plot_eigen_vector, weight_alpha)
    scale_quiver = 1.5;
    [SX,SY,SZ] = sphere(200);

    sphere_xyz = [SX(:) SY(:) SZ(:)]';
    colormap_sphere = zeros(size(SX));

    % color the sphere according to the pdf of the Bingham
    for ix=1:size(SX,1)
        for iy=1:size(SY,1)
            x = SX(ix, iy);
            y = SY(ix, iy);
            z = SZ(ix, iy);
            colormap_sphere(ix,iy) = bing.pdf([x;y;z]);
        end
    end

    colormap(jet);

    if nargin > 2
        surf(SX,SY,SZ, colormap_sphere, 'EdgeColor', 'none', 'FaceAlpha', weight_alpha);
    else
        surf(SX,SY,SZ, colormap_sphere, 'EdgeColor', 'none', 'FaceAlpha', 0.6);
    end

    hold on;

    mode_plot = scale_quiver * bing.mode;
    quiver3(0,0,0,mode_plot(1),mode_plot(2),mode_plot(3),'linewidth',3, 'color', 'k');
    text(mode_plot(1), mode_plot(2), mode_plot(3),['Mode'],'HorizontalAlignment','left','FontSize',12);

    if is_plot_eigen_vector
        eigen_1 = scale_quiver * bing.M(:,1);
        eigen_2 = scale_quiver * bing.M(:,2);
        quiver3(0,0,0,eigen_1(1),eigen_1(2),eigen_1(3),'linewidth',3, 'color', 'r');
        quiver3(0,0,0,eigen_2(1),eigen_2(2),eigen_2(3),'linewidth',3, 'color', 'b');
        text(eigen_1(1), eigen_1(2), eigen_1(3),['Eigen Vector 1'],'HorizontalAlignment','left','FontSize',12);
        text(eigen_2(1), eigen_2(2), eigen_2(3),['Eigen Vector 2'],'HorizontalAlignment','left','FontSize',12);
    end

    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis vis3d;
    axis equal;
    hold off;

end