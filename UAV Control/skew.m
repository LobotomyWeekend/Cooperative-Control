% Generates a Skew Symmetric Matrix
% Such that Sx*v = cross(Sx,v)
%
% Inputs  : x    [3,1]
% Outputs : Sx   [3,3] 
function Sx = skew(x)
    Sx = [0,    -x(3),  x(2) ;
          x(3),     0,  -x(1);
         -x(2),  x(1),   0  ];
end 