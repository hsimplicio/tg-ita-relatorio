function J = evaluateObjective(z, packInfo, ...
                               boundaryObjective, ...
                               pathObjective)
    % Evaluate the combined objective function
    % Inputs:
    %   z: [nz,1] = decision variables
    %   packInfo: struct = information about the problem
    %   boundaryObjective: function = Mayer term (phi)
    %   pathObjective: function = Lagrange term (L)
    %
    % Outputs:
    %   J = scalar = objective value

    % Unpack z
    [time, state, control] = unpackZ(z, packInfo);

    % Initialize objective
    J = 0;

    % Add Mayer term if provided
    if ~isempty(boundaryObjective)
        phi = boundaryObjective(state(:, 1), ...
                                state(:, end), ...
                                time(1), ...
                                time(end));
        J = J + phi;
    end

    % Add Lagrange term if provided
    if ~isempty(pathObjective)
        % Trapezoidal integration of the path objective
        integrand = pathObjective(time, state, control);
        L = trapz(time, integrand);
        J = J + L;
    end
end
