function [alx] = plot_quaternions_animate(quat, pause_time)

hold off;
class_quat = quaternion(quat); 
alx = PlotRotation(class_quat,pause_time); 
%plot3( squeeze(alx(1,:,:)).', squeeze(alx(2,:,:)).', squeeze(alx(3,:,:)).', '*' );
hold on;
%axis equal;
%axis vis3d;
