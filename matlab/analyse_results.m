%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A script for computing and plotting estimation errors. %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% add the input files as a list
input_files = [
    "input/dance/out_camera.csv" ...
    "input/dance/out_lag_10.csv" ...
    "input/dance/out_batch_processed.csv"
];

% select the marker for which the analysis should be run
marker = "Marker_1";

% set the labels used for plotting
labels = ["Cam. Only", "Fixed-Lag 10 Frames", "Batch"];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data = [];
groups = [];

for i=1:size(input_files, 2)
        input_data = readtable(input_files(1, i), 'Delimiter', ',');
        marker_data = input_data(strcmp(input_data.marker, marker), :);
        truePos = [marker_data{:, 'trueX'} marker_data{:, 'trueY'} marker_data{:, 'trueZ'}];
        estimated = [marker_data{:, 'x'} marker_data{:, 'y'} marker_data{:, 'z'}];
        error = vecnorm((truePos - estimated), 2, 2);
        data = [data; error];
        groups = [groups; repmat({labels(i)}, size(marker_data, 1), 1)];
end

h = boxplot(data, groups);
set(h,{'linew'},{2})
set(h(7,:),'Visible','off')
grid on;
ax = gca;
ax.FontSize = 30; 
ylabel('Error in m', 'FontSize', 28)
ylim([0 7e-3])