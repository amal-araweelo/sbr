function X=axxb_closedform( e_bh, e_sc ) 
    % e_bh:   a 3x7 matrix that contain 3 forward kinematics measurements
    %         obtained from tf_echo. The format of each row must be 
    %         [tx ty tz qx qy qz qw] 
    % e_sc:   a 3x7 matrix that contain 3 AR tag measurements obtained 
    %         from tf_echo. The format of each row must be 
    %         [tx ty tz qx qy qz qw] 
    % return: the 4x4 homogeneous transformation of the hand-eye 
    %         calibration
    
    E = zeros(4,4,3); % E_bh_i
    for i=1:3
        t = e_bh(i,1:3)';           % translation from row i
        q = e_bh(i,4:7);            % quaternion from row i
        q_matlab = [q(4) q(1:3)]; 
        R = quat2rotm(q_matlab);    % convert to q to R
        E(:,:,i)=[ R t; 0 0 0 1];   % build 4x4 homogeneuous transformation
    end

    S = zeros(4,4,3); % E_sc_i
    for i=1:3
        t = e_sc(i,1:3)';           % translation from row i
        q = e_sc(i,4:7);            % quaternion from row i
        q_matlab = [q(4) q(1:3)]; 
        R = quat2rotm(q_matlab);    % convert to q to R
        S(:,:,i)=[ R t; 0 0 0 1];   % build 4x4 homogeneuous transformation
    end

    % Construct A's and B's
    A1 = E(:,:,1) \ E(:,:,2);
    A2 = E(:,:,1) \ E(:,:,3);
    B1 = S(:,:,1) / S(:,:,2);
    B2 = S(:,:,1) / S(:,:,3);

    % Extract rotation matrices
    RA1 = A1(1:3,1:3);
    RA2 = A2(1:3,1:3);

    RB1 = B1(1:3,1:3);
    RB2 = B2(1:3,1:3);

    % Extract translation vectors
    tA1 = A1(1:3,4);
    tA2 = A2(1:3,4);
    tB1 = B1(1:3,4);
    tB2 = B2(1:3,4);
    
    % Calculate alphas and betas (skew symmetric matrices)
    alpha1_hat = logm(RA1);
    alpha2_hat = logm(RA2);
    beta1_hat = logm(RB1);
    beta2_hat = logm(RB2);

    % Alpha and beta vectors
    alpha1 = [alpha1_hat(3,2);alpha1_hat(1,3);alpha1_hat(2,1)];
    alpha2 = [alpha2_hat(3,2);alpha2_hat(1,3);alpha2_hat(2,1)];
    beta1 = [beta1_hat(3,2);beta1_hat(1,3);beta1_hat(2,1)];
    beta2 = [beta2_hat(3,2);beta2_hat(1,3);beta2_hat(2,1)];

    % Curly A and B
    A = [alpha1 alpha2 cross(alpha1, alpha2)];
    B = [beta1 beta2 cross(beta1, beta2)];
    
    % Solve rotation matrix
    RX = A/B;

    % Solve translation
    C = [RA1 - eye(3);RA2 - eye(3)];
    D = [RX*tB1 - tA1;RX*tB2 - tA2];
    tX = C \ D;

    % Contruct X
    X = [ RX tX; 0 0 0 1];

end