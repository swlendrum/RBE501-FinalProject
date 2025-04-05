function S_space = space_screw(S_body, M)
% SPACE_SCREW Converts screw axes from the body frame to the space frame.
%
%   S_space = space_screw(S_body, M) transforms the screw axes defined in 
%   the body frame to the space frame using the adjoint transformation.
%
%   Inputs:
%       S_body - 6×n Matrix of screw axes in the body frame
%       M      - 4×4 Home configuration (end-effector pose at zero position)
%
%   Output:
%       S_space - 6×n Matrix of screw axes in the space frame
%
%   Description:
%   - Uses the adjoint transformation of the home configuration M to 
%     convert body-frame screw axes to the space frame.
%   - The transformation is performed as: S_space = adjoint(M) * S_body.
%
%   See also: adjoint
%
%   Example Usage:
%       S_space = space_screw(S_body, M);

    S_space = adjoint(M) * S_body;
end