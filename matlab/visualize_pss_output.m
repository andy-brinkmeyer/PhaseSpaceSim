%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A script used to create som visualisations. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
path = 'input/cube_collision/';

start = 2500;
length = 1000/3;
endf = start + length;

input_filename = strcat(path, 'out_batch_processed.csv');
input_data = readtable(input_filename, 'Delimiter', ',');
marker_1_data = input_data(strcmp(input_data.marker, 'Marker_1'), :);

input_filename_lag = strcat(path, 'out_lag_10.csv');
input_data_lag = readtable(input_filename_lag, 'Delimiter', ',');
marker_1_data_lag = input_data_lag(strcmp(input_data_lag.marker, 'Marker_1'), :);

input_filename_cam = strcat(path, 'out_camera.csv');
input_data_cam = readtable(input_filename_cam, 'Delimiter', ',');
marker_1_data_cam = input_data_cam(strcmp(input_data_cam.marker, 'Marker_1'), :);

marker_1_data_rm = rmmissing(marker_1_data_lag);
rmse = sqrt(mean((marker_1_data_rm{:, 'y'} - marker_1_data_rm{:, 'trueY'}).^2))

figure()
% for i=1:size(marker_1_data, 1)
%     start = i-1000;
%     if start < 1
%         start = 1;
%     end
%     plot3(marker_1_data{start:i, 'trueX'}, marker_1_data{start:i, 'trueY'}, marker_1_data{start:i, 'trueZ'}, ...
%     marker_1_data{start:i, 'x'}, marker_1_data{start:i, 'y'}, marker_1_data{start:i, 'z'});
%     grid on
%     xlim([-1 1]);
%     ylim([-1 0.5]);
%     zlim([-6 -1]);
%     pause(0.01);
% end
p = plot3(-marker_1_data_cam{:, 'x'}, marker_1_data_cam{:, 'y'}, -marker_1_data_cam{:, 'z'}, 'o', ...
    -marker_1_data_lag{:, 'x'}, marker_1_data_lag{:, 'y'}, -marker_1_data_lag{:, 'z'}, ...
    -marker_1_data_lag{:, 'trueX'}, marker_1_data_lag{:, 'trueY'}, -marker_1_data_lag{:, 'trueZ'});
p(1).LineWidth = 1.5;
p(2).LineWidth = 1.5;
p(3).LineWidth = 1.5;
p(3).Color = 'k';
grid on
ax = gca;
ax.FontSize = 30; 
xlabel('x Position', 'FontSize', 28)
ylabel('y Position', 'FontSize', 28)
zlabel('z Position', 'FontSize', 28)
legend(["Cam. Only", "Incremental", "True Trajectory"])

figure()
plot(marker_1_data{:, 'trueX'}, marker_1_data{:, 'trueY'}, ...
    marker_1_data_lag{:, 'x'}, marker_1_data_lag{:, 'y'})
grid on