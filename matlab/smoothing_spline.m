p = PhaseSpaceSim('input','output');
p.readInput();
smoothingParams = 0.999999 * ones(3, 3);
p.computeKinematics(smoothingParams);
xPos = p.markerKinematics.Marker_2.truePos(:,1);
xVel = p.markerKinematics.Marker_2.trueVel(:,1);
xAcc = p.markerKinematics.Marker_2.trueAcc(:,1);
pos_spline = p.markerKinematics.Marker_2.posSpline{1};
vel_spline = p.markerKinematics.Marker_2.smoothedVel(:,1);
acc_spline = p.markerKinematics.Marker_2.smoothedAcc(:,1);
t = p.markerKinematics.Marker_2.t;
p.createImu();
p.generateImuMeasurements();

% camera metadata
calibratedPositionOffset = [0 0 0; 0.01 0 0];
p.setCameraMetaData(120, 0.1, 0.000001);


% print errors
for j = 1:length(p.metaData.markers)
    marker = p.metaData.markers{j};
    fprintf('Smoothing Error for %s:\n', marker)
    disp(p.markerKinematics.(marker).smoothingError)
end

% plote the data
figure()
subplot(3, 1, 1)
plot(t, xPos, t, pos_spline(t))
legend('True Pos', 'Spline Pos')
grid on
subplot(3, 1, 2)
plot(t(2:end-1), xVel(2:end-1), t, vel_spline)
legend('True Vel', 'Spline Vel')
grid on
subplot(3, 1, 3)
plot(t(3:end-2), xAcc(3:end-2), t, acc_spline)
legend('Centdiff Acc', 'Spline Acc')
grid on
