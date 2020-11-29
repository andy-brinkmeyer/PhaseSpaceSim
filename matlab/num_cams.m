filename = 'output/paper/num_cam/4/cube/measurements.csv';

data = readtable(filename, 'Delimiter', ',');
for i=1:size(data, 1)
    cam = data{i, 'cameras'};
    if cam == ""
       continue 
    end
    data{i, 'cameras'} = cellstr('Camera_1;Camera_2;Camera_3;Camera_4');
end

writetable(data, filename);