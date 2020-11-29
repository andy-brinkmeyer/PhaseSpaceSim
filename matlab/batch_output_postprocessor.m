%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A script for postprocessing the batch results. %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% select the raw batch file to process
dir = 'input/cube_collision/';
batch_filename = 'out_batch.csv';
batch_data = readtable(strcat(dir, batch_filename), 'Delimiter', ',');

% select the camera or fixed-lag output file from the same simulation run
reference_filename = 'out_lag_10.csv';
reference_data = readtable(strcat(dir, reference_filename), 'Delimiter', ',');
reference_data = reference_data(...
    strcmp(reference_data.marker, 'Marker_1'), :);

out = join(batch_data, reference_data, 'Keys', 1, ...
    'LeftVariables', [1,2,3,4,5,6,7,8], 'RightVariables', [2,3,4,5,6,10,11,12,13]);

% set the output path here
writetable(out, strcat(dir, 'out_batch_processed.csv'));