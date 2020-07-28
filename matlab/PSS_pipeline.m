% Script for PSS pipeline. Set the variables at the beginning to the values
% that match your simulation case and run the script.

% Input and output directory. The input files must be named
% measurements.csv and meta.json. The output files will have the same name.
inputPath = 'input';
outputPath = 'output';

% Define the smoothing params, the higher the lower the smoothing. Each row
% i represents the x, y and z direction of the i-th marker as ordered in
% meta.json. If you want to smooth one trajectory individually you can use
% the smoothSingleTrajectory method. 
smoothingParams = 0.999999 * ones(3, 3);
applySmoothing = true;

% Define the IMU parameters. They should be self explanatory.
g = 9.807; % gravity (m/s^2)
accel_range = 16*g; % measurement range (m/s^2)
accel_res = 1/2048 * g; % measurement resolution (m/s^2)
accel_bias = g * [0.003 0.005 -0.004]; % bias (m/s^2)
accel_noise_dens = 160e-6 * g * [1 1 1]; % noise density (m/s^2/sqrt(Hz))

deg_rad = pi/180; % degree to rad conversion
gyro_range = 2000 * deg_rad; % measurement range (rad/s)
gyro_res = 1/16.4 * deg_rad; % measurement resolution (rad/s)
gyro_bias = deg_rad * [0.12 -0.05 -0.1]; % measurement bias (rad/s)
gyro_noise_dens = 0.008 * deg_rad; % noise density (rad/s/sqrt(Hz))

% Calibration offset of the cameras. Each row i represents the x, y and z
% position of the i-th camera according to the ordering in meta.json.
calibratedPositionOffset = zeros(4,3);

% More camera metadata, should be self explanatory.
fieldOfView = 120; % in degree
sensorWidth = 0.1; % width of the camera sensors in m
sensorVariance = 0.000001; % Variance on measurements in m^2.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = PhaseSpaceSim(inputPath, outputPath);
p.readInput();

% set metadata
p.setCameraMetaData(fieldOfView, sensorWidth, sensorVariance, ...
    calibratedPositionOffset);

% compute kinematics
if applySmoothing
    p.computeKinematics(smoothingParams);
    % print smoothing error
    for j = 1:length(p.metaData.markers)
    marker = p.metaData.markers{j};
    fprintf('Smoothing Error for %s:\n', marker)
    disp(p.markerKinematics.(marker).smoothingError)
end
else 
    p.computeKinematics();
end

% create imu
accelParams = accelparams('MeasurementRange', accel_range, ... 
                    'Resolution', accel_res, 'ConstantBias', accel_bias, ...
                    'NoiseDensity', accel_noise_dens);
gyroParams = gyroparams('MeasurementRange', gyro_range, ... 
                    'Resolution', gyro_res, 'ConstantBias', gyro_bias, ...
                    'NoiseDensity', gyro_noise_dens);
p.createImu(accelParams, gyroParams);

% generate measurements
p.generateImuMeasurements(applySmoothing);

