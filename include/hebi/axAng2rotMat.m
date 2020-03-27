function r = axang2rotmat(v, theta, normflag)
%AXANG2ROTMAT Convert rotation from axis-angle to matrix representation.
%   R = AXANG2ROTMAT(V, THETA) returns a 3x3 matrix representation of rotation 
%   defined by the axis-angle rotation vector V and angle THETA.
%
%   R = AXANG2ROTMAT(V, THETA, 'normalized') returns a 3x3 matrix representation
%   of rotation defined by the axis-angle rotation vector V and angle THETA,
%   where V has already be normalized to unit length.
%
%   V is a non-empty 3xN or Nx3 matrix of N rotation vectors. The rotation
%   angle, THETA, (in radians) can be a 1xN or Nx1 vector of rotation angles
%   whose dimensions agree with those of V or a scalar value if all rotation
%   vectors have identical rotation angles. The output R is a 3x3xN numeric
%   array of rotation matrices.
%
%   R = AXANG2ROTMAT(VT) and R = AXANG2ROTMAT(VT, 'normalized') return 3x3xN
%   numeric arrays of N 3x3 matrix representations of rotation defined by the
%   4xN or Nx4 compound axis-angle vector VT where the first three elelments
%   form the axis vector, V, and the fourth is the angle THETA.
%
%   The matrix form of Rodrigues' rotation formula is used to compute an
%   exponential map from so(3) to SO(3). The zero length vector, V = [0 0 0]',
%   is normalized to [1 1 1]'*sqrt(3)/3, such that the resultant rotation matrix
%   is valid: det(R) = 1.
%
%   See also ROTMAT2AXANG, ISROTMAT.

%   Andrew D. Horchler, adh9 @ case . edu, Created 10-16-11
%   Revision: 1.0, 10-16-11


% test input arguments
narginchk(1, 3)

n = size(v);
if nargin == 1
    if ~isreal(v) || ~isnumeric(v)
        error('Input must be real numeric array.')
    end
    
    if ~ismatrix(v) || isempty(v) || ~any(n == 4)
        error('First input argument must be a non-empty Nx4 or 4xN matrix.')
    end
    
    % ensure that matrix of axis-angle vectors is 4xN
    if n(2) == 4 && n(1) ~= 4
        v = v';
        n(2) = n(1);
    end
    
    % assign v and theta separately
    theta = v(4,:);
    v = v(1:3,:);
else
    if ~isreal(v) || ~isreal(theta) || ~isnumeric(v) || ~isnumeric(theta)
        error('Input arguments must be real numeric arrays.')
    end
    
    if ~ismatrix(v) || isempty(v) || ~any(n == 3)
        error('First input argument must be a non-empty Nx3 or 3xN matrix.')
    end
    
    if ~isvector(theta) || isempty(theta)
        error('Second input argument must be a non-empty vector.')
    end

    m = length(theta);
    if ~any(n == m) && m ~= 1
        error('Input dimensions do not agree.')
    end
    
    % ensure that matrix of axis vectors is 3xN and that angle vector is 1xN
    if n(2) == 3 && n(1) ~= 3
        v = v';
        theta = theta';
        n(2) = n(1);
    end
end

% normalize axis vectors unless 'normalized' flag is specified
if nargin == 3
    if ~strcmp(normflag, 'normalized')
        error('Optional third input argument must be string ''normalized''.')
    end
else
	d = 1./sqrt(sum(v.*v, 1));
	v = v.*d([1 1 1], :);
    
    % normalize zero length vector to [1 1 1]'*sqrt(3)/3
	v(:, ~isfinite(d)) = sqrt(3)/3;
end

% build the rotation matrix
s = sin(theta);
c = cos(theta);
t = 1-c;

x = v(1,:);
y = v(2,:);
z = v(3,:);

% yes, this is the fastest way to calculate Rodrigues' rotation formula
r  = zeros(3,3,n(2));
tx = t.*x;
ty = t.*y;
tz = t.*z;
r(1,1,:) = tx.*x+c;    r(1,2,:) = tx.*y-s.*z; r(1,3,:) = tx.*z+s.*y;
r(2,1,:) = ty.*x+s.*z; r(2,2,:) = ty.*y+c;    r(2,3,:) = ty.*z-s.*x;
r(3,1,:) = tz.*x-s.*y; r(3,2,:) = tz.*y+s.*x; r(3,3,:) = tz.*z+c;
