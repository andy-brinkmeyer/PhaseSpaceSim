%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A script for computing and plotting rotation errors. %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% add the input files as a list
input_file = "input/paper/complex_cube_1/out_batch_processed.csv";

% select the marker for which the analysis should be run
marker = "Marker_1";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

input_data = readtable(input_file, 'Delimiter', ',');
marker_data = input_data(strcmp(input_data.marker, marker), :);
rotTrue = quaternion(marker_data{:, 'trueQ0'}, ...
    marker_data{:, 'trueQ1'}, marker_data{:, 'trueQ2'}, ...
    marker_data{:, 'trueQ3'});
rot = quaternion(marker_data{:, 'q0'}, ...
    marker_data{:, 'q1'}, marker_data{:, 'q2'}, ...
    marker_data{:, 'q3'});
rotDiff = quat2axang(rot .* conj(rotTrue));
marker_data{:, 'rotError'} = abs(rotDiff(:, 4));

plot(marker_data{:, 't'}, marker_data{:, 'rotError'});
grid on;
xlabel('Time in s');
ylabel('Rotation Error in rad');