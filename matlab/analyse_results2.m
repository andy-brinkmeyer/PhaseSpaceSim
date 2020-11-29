%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A script for computing and plotting estimation errors. %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% add the input files as a list
input_files_cube = [
    "input/paper/cam_rate/960/dance/out_camera.csv" ...
    "input/paper/cam_rate/960/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/960/dance/out_batch_processed.csv";
    "input/paper/cam_rate/420/dance/out_camera.csv" ...
    "input/paper/cam_rate/420/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/420/dance/out_batch_processed.csv";
    "input/paper/cam_rate/360/dance/out_camera.csv" ...
    "input/paper/cam_rate/360/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/360/dance/out_batch_processed.csv";
    "input/paper/cam_rate/120/dance/out_camera.csv" ...
    "input/paper/cam_rate/120/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/120/dance/out_batch_processed.csv";
    "input/paper/cam_rate/60/dance/out_camera.csv" ...
    "input/paper/cam_rate/60/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/60/dance/out_batch_processed.csv";
    "input/paper/cam_rate/30/dance/out_camera.csv" ...
    "input/paper/cam_rate/30/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/30/dance/out_batch_processed.csv"
];
input_files_dance = [
    "input/paper/cam_rate/960/dance/out_camera.csv" ...
    "input/paper/cam_rate/960/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/960/dance/out_batch_processed.csv";
    "input/paper/cam_rate/420/dance/out_camera.csv" ...
    "input/paper/cam_rate/420/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/420/dance/out_batch_processed.csv";
    "input/paper/cam_rate/360/dance/out_camera.csv" ...
    "input/paper/cam_rate/360/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/360/dance/out_batch_processed.csv";
    "input/paper/cam_rate/120/dance/out_camera.csv" ...
    "input/paper/cam_rate/120/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/120/dance/out_batch_processed.csv";
    "input/paper/cam_rate/60/dance/out_camera.csv" ...
    "input/paper/cam_rate/60/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/60/dance/out_batch_processed.csv";
    "input/paper/cam_rate/30/dance/out_camera.csv" ...
    "input/paper/cam_rate/30/dance/out_lag_10.csv" ...
    "input/paper/cam_rate/30/dance/out_batch_processed.csv"
];

% select the marker for which the analysis should be run
marker = "Marker_1";

groups1 = ["960", "420", "360", "120", "60", "30"];
groups2 = {"Camera", "Fixed Lag [Ours]", "Batch"};
xlabels = {'Cube', 'Dance'};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% create the categories and data vectors
data = {};
grouping = {};
for i=1:size(input_files_cube, 1)
    cell_data = cell(2, size(input_files_cube, 2));
   for j=1:size(input_files_cube, 2)
       path = input_files_cube(i, j);
       input_data = readtable(path, 'Delimiter', ',');
       marker_data = input_data(strcmp(input_data.marker, marker), :);
       truePos = [marker_data{:, 'trueX'} marker_data{:, 'trueY'} marker_data{:, 'trueZ'}];
       estimated = [marker_data{:, 'x'} marker_data{:, 'y'} marker_data{:, 'z'}];
       error = vecnorm((truePos - estimated), 2, 2) * 1000;
       cell_data{1,j} = error;
       
       path = input_files_dance(i, j);
       input_data = readtable(path, 'Delimiter', ',');
       marker_data = input_data(strcmp(input_data.marker, marker), :);
       truePos = [marker_data{:, 'trueX'} marker_data{:, 'trueY'} marker_data{:, 'trueZ'}];
       estimated = [marker_data{:, 'x'} marker_data{:, 'y'} marker_data{:, 'z'}];
       error = vecnorm((truePos - estimated), 2, 2) * 1000;
       cell_data{2,j} = error;
   end
   data{i} = cell_data;
end

figure('Color',[1 1 1],'Position',[178 457 1114 521])
main_ax = axes; % create a tmporary axes
pos = main_ax.Position;
group_number = size(groups1, 2);
width = pos(3)/group_number; % the width of each group
corner = linspace(pos(1),pos(3)+pos(1),group_number+1);
clf % clear the area!
for k = 1:group_number
    % create a different axes for each group:
    ax = axes;
    multiple_boxplot(data{k}, xlabels, groups2); % plot the first set
    legend('Location','northwest')
    % set(h(7,:),'Visible','off')
    % set the ylim to include all data:
    ax.YLim = [-1e-4 2];
    % ax.XTickLabelRotation = 90; % rotate xlables if needed
    ax.FontSize = 20;
    box off
    if k == 1 
        ylabel('Error in mm', 'FontSize', 28) % only for the most right axes 
    else
        ax.YTick = [];
    end
    xlabel(groups1(k))
    ax.Position = [corner(k) 0.2 width 0.7];
end