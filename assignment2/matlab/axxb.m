function X = axxb(e_bh, e_sc)
    % e_bh: a Nx7 matrix that contains N forward kinematics measurements
    %       obtained from tf_echo. The format of each row must be
    %       [tx ty tz qx qy qz qw]
    %
    % e_sc: a Nx7 matrix that contains N AR tag measurements obtained
    %       from tf_echo. The format of each row must be
    %       [tx ty tz qx qy qz qw]
    %
    % return: the 4x4 homogeneous transformation of the hand-eye
    %         calibration

    N = size(e_bh);
    N = N(1);
    % disp(N);

    E = zeros(4,4,N); % E_bh_i
    for i=1:N
        t = e_bh(i,1:3)';           % translation from row i
        q = e_bh(i,4:7);            % quaternion from row i
        q_matlab = [q(4) q(1:3)]; 
        R = quat2rotm(q_matlab);    % convert to q to R
        E(:,:,i)=[ R t; 0 0 0 1];   % build 4x4 homogeneuous transformation
    end

    S = zeros(4,4,N); % E_sc_i
    for i=1:N
        t = e_sc(i,1:3)';           % translation from row i
        q = e_sc(i,4:7);            % quaternion from row i
        q_matlab = [q(4) q(1:3)]; 
        R = quat2rotm(q_matlab);    % convert to q to R
        S(:,:,i)=[ R t; 0 0 0 1];   % build 4x4 homogeneuous transformation
    end
    
    % Construct A and B (reusing E1 and S1)
    A = zeros(4,4,N-1);
    B = zeros(4,4,N-1);

    for i=1:N-1
        A(:,:,i) = E(:,:,1) \ E(:,:,i+1);
        B(:,:,i) = S(:,:,1) / S(:,:,i+1);
    end 

    % Extract RA and RB
    RA = A(1:3,1:3,:); % 3 x 3 x N-1
    RB = B(1:3,1:3,:); % 3 x 3 x N-1

    % Calculate alphas and betas
    alpha_hats = zeros(3,3,N-1);
    beta_hats = zeros(3,3,N-1);
    alphas = zeros(3,N-1);
    betas = zeros(3,N-1);

    for i=1:N-1
        alpha_hats(:,:,i) = logm(RA(1:3,1:3,i));
        beta_hats(:,:,i)= logm(RB(1:3,1:3,i));
        alphas(:,i) = [alpha_hats(3,2,i);alpha_hats(1,3,i);alpha_hats(2,1,i)];
        betas(:,i)=[beta_hats(3,2,i);beta_hats(1,3,i);beta_hats(2,1,i)];
    end
    
    % Solve Rx
    Rx = solveRx(alphas, betas);

    % Extract tA and tB
    tA = zeros(3,N-1); % 3 x N-1
    tB = zeros(3,N-1); % 3 x N-1

    % convert 3 x 1 x N-1 to 3 x N-1
    for i = 1:N-1
        tA(:,i) = A(1:3,4,i); 
        tB(:,i) = B(1:3,4,i);
    end
    % Solve tx
    tx = solveTx(RA,tA,RB,tB, Rx);

    % Construct X
    X = [ Rx tx; 0 0 0 1];
end