% setup
clear;
clc;

%% Test
[e_bh, e_sc, X_true] = generatedata(10);

X_est = axxb(e_bh, e_sc);

disp(X_true)
disp(X_est)

error = norm(X_true-X_est);

%% Test simulation
% open data.txt
fid = fopen('data.txt','r');

% skip "# gripper"
fgetl(fid);

% store gripper measurements
e_bh = [];

% read until we reach "# marker"
while true
    line = strtrim(fgetl(fid));
    if strcmp(line, '# marker')
        break;
    end
    % ignore empty lines
    if ~isempty(line)
        values = sscanf(line,'%f')';
        e_bh = [e_bh; values];
    end
end

e_sc = [];
while ~feof(fid)
    line = strtrim(fgetl(fid));
    if ~isempty(line)
        values = sscanf(line,'%f')';
        e_sc = [e_sc; values];
    end
end
fclose(fid);

% run hand eye calibration
X = axxb( e_bh, e_sc )

%% Expert challenge - error check

%ros2 run tf2_ros tf2_echo gripper_pick camera_link -p 10
% X_true= [ 1.0000000000 -0.0000036732 -0.0000036732 -0.0324999930;
%     0.0000036732 1.0000000000 -0.0000000000 -0.0625001194;
%     0.0000036732 -0.0000000000 1.0000000000 -0.1644001194;
%     0.0000000000 0.0000000000 0.0000000000 1.0000000000];
X_true =  [1.000 -0.000 -0.000 -0.032;
0.000  1.000 -0.000 -0.063;
0.000 -0.000  1.000 -0.164;
0.000  0.000  0.000  1.000];

% translation error
t_true = X_true(1:3,4);
t_est  = X(1:3,4);

translation_error = norm(t_true-t_est);
fprintf("Translation error in mm: %.6f\n", translation_error*1000)

% rotation error
R_true = X_true(1:3,1:3);
R_est  = X(1:3,1:3);
R_error = R_true*R_est';

theta = acos((trace(R_error)-1)/2);
fprintf("Rotation error in rad: %.6f\n", theta)


