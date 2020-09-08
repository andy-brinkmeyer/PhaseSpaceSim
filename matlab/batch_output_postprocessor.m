batch_filename = 'input/cube_occlusion/out_batch.csv';
batch_data = readtable(batch_filename, 'Delimiter', ',');

reference_filename = 'input/cube_occlusion/out_camera.csv';
reference_data = readtable(reference_filename, 'Delimiter', ',');

out = join(batch_data, reference_data, 'Keys', 1, ...
    'LeftVariables', [1,2,3,4], 'RightVariables', [3,4,5,6]);

writetable(out, 'input/cube_occlusion/out_batch_processed.csv');