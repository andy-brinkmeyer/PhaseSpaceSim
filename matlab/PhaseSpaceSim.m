classdef PhaseSpaceSim < handle
    %PHASESPACESIM Class that encapsulates methods needed for generating
    %simulated measurements from the Phasespace system.
    %   USe this class to generate Measurements from the Phasespace camera
    %   system and from the IMUs.
    
    properties
        inputTable
        metaData
        markerKinematics
        accelParams
        gyroParams
        imu
    end
    
    properties (Access = private)
        inputDir
        outputDir
    end
    
    methods
        function obj = PhaseSpaceSim(inputDir, outputDir)
            %PHASESPACESIM Construct an instance of this class
            %   Define the input and output dir. The input files must be
            %   located in the input dir and need to be called
            %   measurements.csv and meta.json
            obj.inputDir = inputDir;
            obj.outputDir = outputDir;
            obj.markerKinematics = struct;
        end
        
        function readInput(obj)
            % read measurements
            inputFile = strcat(obj.inputDir, '/measurements.csv');
            obj.inputTable = readtable(inputFile, 'Delimiter', ',');
            
            % read metadata
            metaFile = strcat(obj.inputDir, '/meta.json');
            obj.metaData = jsondecode(fileread(metaFile));
        end
        
        function setCameraMetaData(obj, fieldOfView, sensorWidth,...
                sensorVariance, ...
                calibratedPositionOffset, calibratedRotation)
            for i = 1:size(obj.metaData.cameras, 1)
               obj.metaData.cameras(i).fieldOfView = ...
                   fieldOfView;
               obj.metaData.cameras(i).sensorWidth = ...
                   sensorWidth;
               obj.metaData.cameras(i).sensorVariance = ...
                   sensorVariance;
               
               % set calibrated camera pose
               if nargin > 4 
                   obj.metaData.cameras(i).calibratedPosition.x = ...
                       obj.metaData.cameras(i).position.x + ...
                       calibratedPositionOffset(i,1);
                   obj.metaData.cameras(i).calibratedPosition.y = ...
                       obj.metaData.cameras(i).position.y + ...
                       calibratedPositionOffset(i,2);
                   obj.metaData.cameras(i).calibratedPosition.z = ...
                       obj.metaData.cameras(i).position.z + ...
                       calibratedPositionOffset(i,3);
               end
               if nargin > 5
                   obj.metaData.cameras(i).calibratedRotation.q0 = ...
                       calibratedRotation(i,1);
                   obj.metaData.cameras(i).calibratedRotation.q1 = ...
                       calibratedRotation(i,2);
                   obj.metaData.cameras(i).calibratedRotation.q2 = ...
                       calibratedRotationOffset(i,3);
                   obj.metaData.cameras(i).calibratedRotation.q3 = ...
                       calibratedRotationOffset(i,4);
               end
            end
            metaFile = strcat(obj.outputDir, '/meta.json');
            jsonData = jsonencode(obj.metaData);
            outFile = fopen(metaFile, 'w');
            fprintf(outFile, jsonData);
            fclose(outFile);
        end
        
        function computeKinematics(obj, smoothingParams)
            for j = 1:length(obj.metaData.markers)
                marker = obj.metaData.markers{j};
                markerData = obj.inputTable(...
                    strcmp(obj.inputTable.marker_id, marker), :);
                
                % assign the input values to object
                obj.markerKinematics.(marker).t = markerData{:, 't'};
                obj.markerKinematics.(marker).truePos = ...
                    markerData{:, {'x_true', 'y_true', 'z_true'}};
                obj.markerKinematics.(marker).trueRot = ...
                    conj(...
                        quaternion(markerData{:, ...
                            {'q0', 'q1', 'q2', 'q3'}}));
                obj.markerKinematics.(marker).frames = ...
                    markerData{:, {'frame'}};
                obj.markerKinematics.(marker).cameras = ...
                    markerData{:, {'cameras'}};
                        
                % get dt
                obj.markerKinematics.(marker).dt = ...
                    diff(markerData{:, 't'});
                
                % compute velocity and acceleration
                obj.markerKinematics.(marker).trueVel = ...
                    centdiff(obj.markerKinematics.(marker).truePos, ...
                    obj.markerKinematics.(marker).t);
                obj.markerKinematics.(marker).trueAcc = ...
                    centdiff(obj.markerKinematics.(marker).trueVel, ...
                    obj.markerKinematics.(marker).t);   
                
                if nargin > 1
                    obj.markerKinematics.(marker).posSpline = {};
                    obj.markerKinematics.(marker).smoothingError = ...
                        zeros(3, 1);
                    for k = 1:3
                    % compute smoothed kinematics
                    obj.markerKinematics.(marker).posSpline{k} = ...
                        fit(obj.markerKinematics.(marker).t, ...
                        obj.markerKinematics.(marker).truePos(:, k), ...
                        'smoothingspline', ... 
                        'SmoothingParam', smoothingParams(j, k));
                    obj.markerKinematics.(marker).smoothingError(k) = obj.RMSE(...
                        obj.markerKinematics.(marker).posSpline{k}(...
                            obj.markerKinematics.(marker).t), ...
                        obj.markerKinematics.(marker).truePos(:, k));
                    [obj.markerKinematics.(marker).smoothedVel(:, k), ...
                        obj.markerKinematics.(marker).smoothedAcc(:, k)] = ...
                        differentiate(...
                            obj.markerKinematics.(marker).posSpline{k}, ...
                            obj.markerKinematics.(marker).t);
                    end
                end
                
                % compute angvel
                obj.markerKinematics.(marker).trueAngvel = ...
                    quat2angvel(obj.markerKinematics.(marker).trueRot, ...
                    obj.markerKinematics.(marker).dt);
            end
        end
        
        function createImu(obj, accelParams, gyroParams)
            if nargin == 1
                g = 9.807;
                accel_range = 16*g;
                accel_res = 1/2048 * g;
                accel_bias = g * [0.003 0.005 -0.004];
                accel_noise_dens = 160e-6 * g * [1 1 1];
                obj.accelParams = accelparams('MeasurementRange', accel_range, ... 
                    'Resolution', accel_res, 'ConstantBias', accel_bias, ...
                    'NoiseDensity', accel_noise_dens);

                deg_rad = pi/180;
                gyro_range = 2000 * deg_rad;
                gyro_res = 1/16.4 * deg_rad;
                gyro_bias = deg_rad * [0.12 -0.05 -0.1];
                gyro_noise_dens = 0.008 * deg_rad;
                obj.gyroParams = gyroparams('MeasurementRange', gyro_range, ... 
                    'Resolution', gyro_res, 'ConstantBias', gyro_bias, ...
                    'NoiseDensity', gyro_noise_dens);
            else
               obj.accelParams = accelParams;
               obj.gyroParams = gyroParams;
            end
            imu_type = 'accel-gyro';
            sample_rate = obj.metaData.samplingRate;
            obj.imu = imuSensor(imu_type, 'SampleRate', sample_rate);
        end
        
        function [imuMeasurements] = generateImuMeasurements(obj)
            imuMeasurements = struct;
            tableColumns = {'frame', 't', 'markerID', 'cameras', ...
                'x', 'y', 'z', 'q0', 'q1', 'q2', 'q3', ...
                'ax', 'ay', 'az', 'wx', 'wy', 'wz'};
            outputTable = cell2table(cell(0,17), 'VariableNames', tableColumns);
            for i = 1:length(obj.metaData.markers)
                marker = obj.metaData.markers{i};
                rot = obj.markerKinematics.(marker).trueRot;
                accBody = rotateframe(rot, ...
                    obj.markerKinematics.(marker).trueAcc);
                angVelBody = rotateframe(rot, ...
                    obj.markerKinematics.(marker).trueAngvel);
                [accReadings, gyroReadings] = obj.imu(accBody, ...
                    angVelBody, rot);
                imuMeasurements.(marker).t = ...
                    obj.markerKinematics.(marker).t;
                imuMeasurements.(marker).accReadings = accReadings;
                imuMeasurements.(marker).gyroReadings = gyroReadings;
                
                % generate output table
                [q0, q1, q2, q3] = parts(obj.markerKinematics.(marker).trueRot);
                markerTable = table(...
                    obj.markerKinematics.(marker).frames, ...
                    obj.markerKinematics.(marker).t, ...
                    char(ones(size(accReadings,1),1) * marker), ...
                    obj.markerKinematics.(marker).cameras, ...
                    obj.markerKinematics.(marker).truePos(:,1), ...
                    obj.markerKinematics.(marker).truePos(:,2), ...
                    obj.markerKinematics.(marker).truePos(:,3), ...
                    q0, q1, q2, q3, ...
                    accReadings(:,1), ...
                    accReadings(:,2), ...
                    accReadings(:,3), ...
                    gyroReadings(:,1), ...
                    gyroReadings(:,2), ...
                    gyroReadings(:,3));
                markerTable.Properties.VariableNames = tableColumns;
                outputTable = [outputTable; markerTable];
            end
            
            % write data to file
            writetable(sortrows(outputTable), strcat(obj.outputDir, ...
                '/','measurements.csv'));
        end
        
        function rmse = RMSE(~, x, xTrue)
           rmse = sqrt(mean((x - xTrue).^2));
        end
    end
end

