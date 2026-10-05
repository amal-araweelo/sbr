function Rx = solveRx(alphas, betas)
    % alphas: A 3xN matrix containing the alpha vectors.
    %         alphas = [alpha1 ... alphaN]
    %         Each alpha_i is a 3x1 vector.
    %
    % betas:  A 3xN matrix containing the beta vectors.
    %         betas = [beta1 ... betaN]
    %         Each beta_i is a 3x1 vector.
    %
    % return: The least squares solution for the 3x3 rotation matrix Rx.

    N = size(alphas,2); % number of cols
    M = zeros(3,3);

    for i=1:N
        alpha_i = alphas(:,i);
        beta_i = betas(:,i);
        M = M + (beta_i * alpha_i');
    end

    % Calculate Rx
    Rx = (M' * M)^(-1/2)*M';

end