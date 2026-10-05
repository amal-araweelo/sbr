% Generate synthetic data to test hand-eye calibration.
% e_bh and e_sc are Nx7 matrices that represent N E_bh and N E_sc 
% transformations. Each row is of the form [ tx ty tz qx qy qz qw ] 
% were tx, ty and tz denote a translation and qx, qy, qz, qw a 
% quaternion. 
% X is a randomly generated hand-eye transformation 
function [e_bh, e_sc, X] = generatedata(N) 
    % Assume we know the transformation between the world and the 
    % checkerboard
    E_bc = [ eye(3) [ 1; 0; 0 ]; 0 0 0 1 ]; 
    % Create a random X for generating the data
    X = randSE3(); % random SE(3) matrix
    e_bh = [];
    e_sc = [];
    for i=1:N
        % Now that you have X and E_bc
        % TODO: Generate a random E_bh, you can use rotm2quat to convert
            % the rotation matrix to a quaternion (be careful because it 
            % outputs in the format [qw, qx, qy, qz]) and append the 
            % transformation to e_bh
        E_bh = randSE3();
        R = E_bh(1:3,1:3); %(rows, cols)
        t = E_bh(1:3,4);
        q = rotm2quat(R); %[qw, qx, qy, qz]
        e_bh = [e_bh; t' q(2) q(3) q(4) q(1)]; %[tx, ty, tx, qx, qy, qz, qw]

        % Now that you have X, E_bc and E_bh
        % TODO: Find E_sc and append the transformation to e_sc
        XE_sc = E_bh \ E_bc;
        E_sc = X \ XE_sc;
        R = E_sc(1:3,1:3);
        t = E_sc(1:3, 4);
        q = rotm2quat(R); %[qw, qx, qy, qz]
        e_sc = [e_sc; t' q(2) q(3) q(4) q(1)]; %[tx, ty, tz, qx, qy, qz, qw]
    end
end

% Generate a random SE3 transformation 
function Rt = randSE3() 
    % TODO: Generate a random rotation matrix 
    q = randn(1, 4);    % random quarternion
    q = q/norm(q);      % random unit quarternion
    R = quat2rotm(q); 
    % disp(R'*R);       % test
    % TODO: Generate a random translation 
    t = randn(3,1); 
    Rt = [ R t; 0 0 0 1];
end