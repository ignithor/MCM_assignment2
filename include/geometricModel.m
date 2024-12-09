%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end
        function updateDirectGeometry(self, q)
            %%% GetDirectGeometryFunction
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
            
            self.q = q;
            self.iTj = self.iTj_0;

            for i=1:self.jointNumber
                T = eye(4);
                if self.jointType(i)
                    T= [1,0,0,0;
                        0,1,0,0;
                        0,0,1,q(i);
                        0,0,0,1];
                elseif ~self.jointType(i)
                    cq = cos(q(i));
                    sq = sin(q(i));
                    T(1:4, 1:4) = [cq, -sq, 0, 0;
                                    sq, cq, 0, 0;
                                    0, 0, 1, 0;
                                    0,0,0,1];

                else
                    error("Joint Type not rotation or prismastic")
                end
                self.iTj(:,:,i) = self.iTj_0(:,:,i) * T;

            end
        end
        function [bTk] = getTransformWrtBase(self,k)
            %% GetTransformatioWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.
            
            if k < 1 || k > self.jointNumber
                error('Joint index out of range');
            end
            
            bTk = eye(4); % Start with identity matrix
            
            for i = 1:k
                bTk = bTk * self.iTj(:,:,i);
            end
        end

    end
end

