% dance 1000: 958, 1999, 2999, 1

path = 'input/cube_collision/';
ref_path = 'output/cube_collision/';
startf = 1;
endf = 2719;

startocc = 958;
ref_start_occ = 1999;
ref_end = 5999;
duration = 4;

input_filename = strcat(path, 'out_lag_10.csv');
input_data = readtable(input_filename, 'Delimiter', ',');
marker_data = input_data(strcmp(input_data.marker, 'Marker_1'), :);

ref_filename = strcat(ref_path, 'measurements.csv');
ref_data = readtable(ref_filename, 'Delimiter', ',');

truePos = [marker_data{startf:endf, 'trueX'} marker_data{startf:endf, 'trueY'} marker_data{startf:endf, 'trueZ'}];
estimated = [marker_data{startf:endf, 'x'} marker_data{startf:endf, 'y'} marker_data{startf:endf, 'z'}];
error = vecnorm((truePos - estimated), 2, 2);

input_filename2 = strcat(path, 'out_batch_processed_vel.csv');
input_data2 = readtable(input_filename2, 'Delimiter', ',');
marker_data2 = input_data2(strcmp(input_data2.marker, 'Marker_1'), :);

truePos2 = [marker_data2{startf:endf, 'trueX'} marker_data2{startf:endf, 'trueY'} marker_data2{startf:endf, 'trueZ'}];
estimated2 = [marker_data2{startf:endf, 'x'} marker_data2{startf:endf, 'y'} marker_data2{startf:endf, 'z'}];
error2 = vecnorm((truePos2 - estimated2), 2, 2);
% interpolate using const. velocity
pos_occ = estimated(958, :);
vel_occ = [ref_data{ref_start_occ, 'vx'} ref_data{ref_start_occ, 'vy'} ref_data{ref_start_occ, 'vz'}];
dt = transpose(0:1/1000:duration);
inter_pos = (vel_occ .* dt) + pos_occ;
ref_pos = [ref_data{ref_start_occ:ref_end, 'x'} ref_data{ref_start_occ:ref_end, 'y'} ref_data{ref_start_occ:ref_end, 'z'}];
ref_error = vecnorm((ref_pos - inter_pos), 2, 2);
ref_t = ref_data{ref_start_occ, 't'}:1/1000:(duration + ref_data{ref_start_occ, 't'});

% h = plot(marker_data{startf:endf, 't'} - 1, error * 1000, ref_t - 1, ref_error * 1000);
h = plot(marker_data{startf:endf, 't'} - 1, error * 1000, dt +1, ref_error * 1000);
grid on
h(1).LineWidth = 4;
h(2).LineWidth = 4;
grid on;
ax = gca;
ax.FontSize = 50; 
set(gca, 'YScale', 'log')
xlabel('time in s', 'FontSize', 60)
ylabel('Error in mm', 'FontSize', 60)
legend('Our System', 'Const. Veloc. Interpolation')