% ---------------------------------------------------------------------
% Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
% Version:    2021-03-18 12:09:55
% Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
% License:    BSD
% ---------------------------------------------------------------------

% Software License Agreement (BSD License)
% Copyright (c) 2021, Computer Science Institute VI, University of Bonn
% All rights reserved.
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
% 
% * Redistributions of source code must retain the above copyright
%   notice, this list of conditions and the following disclaimer.
% * Redistributions in binary form must reproduce the above
%   copyright notice, this list of conditions and the following
%   disclaimer in the documentation and/or other materials provided
%   with the distribution.
% * Neither the name of University of Bonn, Computer Science Institute
%   VI nor the names of its contributors may be used to endorse or
%   promote products derived from this software without specific
%   prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
% COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%% --------------------------------------------------------------------

function [n] = select_type(P_wayp,V_wayp,A_wayp) %#codegen


select = 0;

if ~isnan(P_wayp)
    select = 100;   
end 

if ~isnan(V_wayp)
    select = select+10;   
end 

if ~isnan(A_wayp)
    select = select+1;   
end    

switch select
    
    case 0
        n = 0;
    case 1
        n = 5;   
    case 10
        n = 6;     
    case 11
        n = 4;         
    case 100
        n = 7;
    case 101
        n = 3;     
    case 110
        n = 2;          
    case 111
        n = 1;  
    otherwise
        n=0;    
end        
        

end