%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A script for postprocessing the batch results. %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% select the raw batch file to process
batch_filename = 'input/complex_cube_occlusion/out_batch.csv';
batch_data = readtable(batch_filename, 'Delimiter', ',');

% select the camera or fixed-lag output file from the same simulation run
reference_filename = 'input/complex_cube_occlusion/out_camera.csv';
reference_data = readtable(reference_filename, 'Delimiter', ',');

out = join(batch_data, reference_data, 'Keys', 1, ...
    'LeftVariables', [1,2,3,4], 'RightVariables', [3,4,5,6]);

% set the output path here
writetable(out, 'input/complex_cube_occlusion/out_batch_processed.csv');