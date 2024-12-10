%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix
        
        % Get the end-effector position with respect to the base
        bTe = self.gm.getTransformWrtBase(self.gm.jointNumber); % Base to end-effector transformation
        p_e = bTe(1:3, 4); % Extract the position of the end-effector


        for i=1:self.gm.jointNumber
            bTi = self.gm.getTransformWrtBase(i);
            % Extract the position and Z-axis of joint i
            p_i = bTi(1:3, 4);  % Position of joint i in base frame
            z_i = bTi(1:3, 3);  % Z-axis of joint i in base frame
            if self.gm.jointType(i)
                % Prismatic joint
                self.J(1:3,i) = [0;0;0];
                self.J(4:6, i) = z_i;
            elseif ~self.gm.jointType(i)
                % Rotation joint
                self.J(1:3, i) = z_i;
                self.J(4:6, i) = cross(z_i, p_e - p_i);
            else
                error("Joint Type not pismatic or rotation")
            end
        end

            
        end
    end
end

