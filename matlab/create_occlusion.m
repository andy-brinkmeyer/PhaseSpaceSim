filename = 'output/cube_collision/measurements.csv';
occlusion_length = 3402;
start = 1;

occlusion_end = start + occlusion_length;
data = readtable(filename, 'Delimiter', ',');
data{:, 'cameras'} = cellstr(strings(occlusion_length + 1, 1));

writetable(data, filename);