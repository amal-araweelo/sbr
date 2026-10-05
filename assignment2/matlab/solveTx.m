function tx=solveTx( RA, tA, RB, tB, RX ) 
    % RA: a 3x3xN matrix with all the rotations matrices RAi
    % tA: a 3xN matrix with all the translation vectors tAi 
    % RB: a 3x3xN matrix with all the rotations matrices 𝑅Bi 
    % tB: a 3xN matrix with all the translation vectors tBi
    % RX: the 3x3 rotation matrix Rx 
    % return: the 3x1 translation vector tx

    
    N = size(RA);
    N = N(3);
    C = [];
    D = [];
    % Least squares
    for i=1:N
        C = [C; eye(3) - RA(:,:,i)];
        D = [D; tA(:,i)- RX*tB(:,i)];
    end

    tx = C \ D;
end