function [c, ceq] = boundaryConstraints(x0, xF, t0, tF, boundaryConditions)
    % Evaluates boundary constraints for the EVTOL problem
    %
    % Inputs:
    %   x0: [5,1] = initial state vector [sx; sy; vx; vy; E]
    %   xF: [5,1] = final state vector [sx; sy; vx; vy; E]
    %   t0: double = initial time
    %   tF: double = final time
    %
    % Outputs:
    %   c: inequality constraints at boundaries
    %   ceq: equality constraints at boundaries

    arguments
        x0 double
        xF double
        t0 {mustBeNumeric}
        tF {mustBeNumeric}
        boundaryConditions struct
    end

    % Initial conditions
    initialPosX = x0(1);
    initialPosY = x0(2);
    initialVelX = x0(3);
    initialVelY = x0(4);
    initialEnergy = x0(5);

    % Final conditions
    finalPosX = xF(1);
    finalPosY = xF(2);
    finalVelX = xF(3);
    finalVelY = xF(4);

    % Equality constraints
    ceq = [
           % Initial conditions
           % x(0) = boundaryConditions.xLow(1)
           initialPosX - boundaryConditions.xLow(1)
           % y(0) = boundaryConditions.xLow(2)
           initialPosY - boundaryConditions.xLow(2)
           % vx(0) = boundaryConditions.xLow(3)
           initialVelX - boundaryConditions.xLow(3)
           % vy(0) = boundaryConditions.xLow(4)
           initialVelY - boundaryConditions.xLow(4)
           % E(0) = boundaryConditions.xLow(5)
           initialEnergy - boundaryConditions.xLow(5)

           % Final conditions
           % x(T) = boundaryConditions.xF(1)
           finalPosX - boundaryConditions.xF(1)
           % y(T) = boundaryConditions.xF(2)
           finalPosY - boundaryConditions.xF(2)
           % vx(T) = boundaryConditions.xF(3)
           finalVelX - boundaryConditions.xF(3)
           % vy(T) = boundaryConditions.xF(4)
           finalVelY - boundaryConditions.xF(4)

           % Time bounds
           t0 - boundaryConditions.t0
           tF - boundaryConditions.tF
          ];

    % No inequality constraints
    c = [];
end
