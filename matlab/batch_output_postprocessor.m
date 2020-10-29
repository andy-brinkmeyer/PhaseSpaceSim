%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A script for postprocessing the batch results. %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% select the raw batch file to process
batch_filename = 'input/dance/out_batch.csv';
batch_data = readtable(batch_filename, 'Delimiter', ',');

% select the camera or fixed-lag output file from the same simulation run
reference_filename = 'input/dance/out_lag_10.csv';
reference_data = readtable(reference_filename, 'Delimiter', ',');
reference_data = reference_data(...
    strcmp(reference_data.marker, 'Marker_1'), :);

out = join(batch_data, reference_data, 'Keys', 1, ...
    'LeftVariables', [1,2,3,4,5,6,7,8], 'RightVariables', [2,3,4,5,6,10,11,12,13]);

% set the output path here
writetable(out, 'input/dance/out_batch_processed.csv');