 omega = [0 0 pi];
 
 r = [0.5 0 0];
 
 f = 1000;
 t_max = 5;
 n = (t_max / (1/f)) + 1;
 ns = transpose(0:n-1);
 t = 0:1/f:t_max;
 
 angvel = ones(n, 3) .*omega;
 omegas = ones(n,3) .* omega;
 
 phi = transpose(t) .* omega;
 R = eul2rotm(phi, 'XYZ');
 quats = rotm2quat(R);
 
 camera_string = "Camera_1;Camera_2;Camera_3;Camera_4;Camera_5;Camera_6;Camera_7;Camera_8";
 marker_id = "Marker_1"; 
 
 pos = zeros(n, 3);
 a = zeros(n, 3);
 aB = zeros(n,3);
 markerVec = [marker_id];
 cameraVec = [camera_string];
 counter = 0;
 for i = 1:n
    pos(i,:) = transpose(transpose(R(:,:,i)) * transpose(r));
    a(i,:) = cross(omega, cross(omega, pos(i,:)));
    aB(i,:) = transpose(R(:,:,i) * transpose(a(i,:))); % check if pos is computed correctly! Rt maps global to body
    if counter == 2
        cameraVec(i,1) = "";
        counter = -1;
    else
        cameraVec(i,1) = camera_string;
    end
    markerVec(i,1) = marker_id;
    counter = counter + 1;
 end
 
 varNames = ["frame","t","marker_id","cameras","x_true", ...
     "y_true","z_true","q0","q1","q2","q3"];
 outputTable = table(...
    ns, transpose(t), markerVec, cameraVec, ...
    pos(:,1), pos(:,2), pos(:,3), ...
    quats(:,1), -quats(:,2), -quats(:,3), -quats(:,4), ...
    'VariableNames', varNames);

writetable(outputTable, "input/analytic_cube/measurements.csv");