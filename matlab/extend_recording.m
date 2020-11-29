% 7588 for dance, 4000 for cube
filename = 'input/paper/cam_rate/30/cube/measurements.csv';
period_length = 4000;
period_dt = 4;
n_periods = 4;

data = readtable(filename, 'Delimiter', ',');
period = data(1:period_length, :);
out_table = period;
for i=2:n_periods
   added_period = period;
   added_period{:, 'frame'} = period{:, 'frame'} + ((i-1) * period_length);
   added_period{:, 't'} = period{:, 't'} + ((i-1) * period_dt);
   out_table = [out_table; added_period];
end

writetable(out_table, filename);



