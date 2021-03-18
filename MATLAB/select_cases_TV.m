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

function cases = select_cases_TV(n,cond,b_sync_V) %#codegen

    persistent caselut;
    if isempty(caselut)
        caselut = cell(7,32);
        caselut{1,1} = int32([10104, 10105, 10107, 10108, 11101, 11103, 11105, 11107, 11108]);
        caselut{1,2} = int32([10103, 10106, 10107, 10108, 11102, 11104, 11106, 11107, 11108]);
        caselut{1,3} = int32([10107, 10108, 11107, 11108]);
        caselut{1,4} = int32([10101, 10102, 10103, 10104, 10105, 10106, 10107, 10108, 11101, 11102, 11103, 11104, 11105, 11106, 11107, 11108]);
        caselut{1,5} = int32([10104, 10105, 10107, 10108, 11101, 11103, 11105, 11107, 11108]);
        caselut{1,6} = int32([10103, 10106, 10107, 10108, 11102, 11104, 11106, 11107, 11108]);
        caselut{1,7} = int32([10107, 10108, 11107, 11108]);
        caselut{1,8} = int32([10101, 10102, 10103, 10104, 10105, 10106, 10107, 10108, 11101, 11102, 11103, 11104, 11105, 11106, 11107, 11108]);
        caselut{1,9} = int32([10104, 10105, 10107, 10108, 11101, 11103, 11105, 11107, 11108]);
        caselut{1,10} = int32([10103, 10106, 10107, 10108, 11102, 11104, 11106, 11107, 11108]);
        caselut{1,11} = int32([10107, 10108, 11107, 11108]);
        caselut{1,12} = int32([10101, 10102, 10103, 10104, 10105, 10106, 10107, 10108, 11101, 11102, 11103, 11104, 11105, 11106, 11107, 11108]);
        caselut{1,13} = int32([10104, 10105, 10107, 10108, 11101, 11103, 11105, 11107, 11108]);
        caselut{1,14} = int32([10103, 10106, 10107, 10108, 11102, 11104, 11106, 11107, 11108]);
        caselut{1,15} = int32([10107, 10108, 11107, 11108]);
        caselut{1,16} = int32([10101, 10102, 10103, 10104, 10105, 10106, 10107, 10108, 11101, 11102, 11103, 11104, 11105, 11106, 11107, 11108]);
        caselut{1,17} = int32([10104, 10108, 11101, 11103, 11105, 11107]);
        caselut{1,18} = int32([10106, 10108, 11107]);
        caselut{1,19} = int32([10108, 11107]);
        caselut{1,20} = int32([10102, 10104, 10106, 10108, 11101, 11103, 11105, 11107]);
        caselut{1,21} = int32([10104, 10108, 11101, 11103, 11105, 11107]);
        caselut{1,22} = int32([10106, 10108, 11107]);
        caselut{1,23} = int32([10108, 11107]);
        caselut{1,24} = int32([10102, 10104, 10106, 10108, 11101, 11103, 11105, 11107]);
        caselut{1,25} = int32([10105, 10107, 11108]);
        caselut{1,26} = int32([10103, 10107, 11102, 11104, 11106, 11108]);
        caselut{1,27} = int32([10107, 11108]);
        caselut{1,28} = int32([10101, 10103, 10105, 10107, 11102, 11104, 11106, 11108]);
        caselut{1,29} = int32([10105, 10107, 11108]);
        caselut{1,30} = int32([10103, 10107, 11102, 11104, 11106, 11108]);
        caselut{1,31} = int32([10107, 11108]);
        caselut{1,32} = int32([10101, 10103, 10105, 10107, 11102, 11104, 11106, 11108]);
        caselut{2,1} = int32([]);
        caselut{2,2} = int32([]);
        caselut{2,3} = int32([]);
        caselut{2,4} = int32([]);
        caselut{2,5} = int32([]);
        caselut{2,6} = int32([]);
        caselut{2,7} = int32([]);
        caselut{2,8} = int32([]);
        caselut{2,9} = int32([]);
        caselut{2,10} = int32([]);
        caselut{2,11} = int32([]);
        caselut{2,12} = int32([]);
        caselut{2,13} = int32([]);
        caselut{2,14} = int32([]);
        caselut{2,15} = int32([]);
        caselut{2,16} = int32([]);
        caselut{2,17} = int32([]);
        caselut{2,18} = int32([]);
        caselut{2,19} = int32([]);
        caselut{2,20} = int32([]);
        caselut{2,21} = int32([]);
        caselut{2,22} = int32([]);
        caselut{2,23} = int32([]);
        caselut{2,24} = int32([]);
        caselut{2,25} = int32([]);
        caselut{2,26} = int32([]);
        caselut{2,27} = int32([]);
        caselut{2,28} = int32([]);
        caselut{2,29} = int32([]);
        caselut{2,30} = int32([]);
        caselut{2,31} = int32([]);
        caselut{2,32} = int32([]);
        caselut{3,1} = int32([]);
        caselut{3,2} = int32([]);
        caselut{3,3} = int32([]);
        caselut{3,4} = int32([]);
        caselut{3,5} = int32([]);
        caselut{3,6} = int32([]);
        caselut{3,7} = int32([]);
        caselut{3,8} = int32([]);
        caselut{3,9} = int32([]);
        caselut{3,10} = int32([]);
        caselut{3,11} = int32([]);
        caselut{3,12} = int32([]);
        caselut{3,13} = int32([]);
        caselut{3,14} = int32([]);
        caselut{3,15} = int32([]);
        caselut{3,16} = int32([]);
        caselut{3,17} = int32([]);
        caselut{3,18} = int32([]);
        caselut{3,19} = int32([]);
        caselut{3,20} = int32([]);
        caselut{3,21} = int32([]);
        caselut{3,22} = int32([]);
        caselut{3,23} = int32([]);
        caselut{3,24} = int32([]);
        caselut{3,25} = int32([]);
        caselut{3,26} = int32([]);
        caselut{3,27} = int32([]);
        caselut{3,28} = int32([]);
        caselut{3,29} = int32([]);
        caselut{3,30} = int32([]);
        caselut{3,31} = int32([]);
        caselut{3,32} = int32([]);
        caselut{4,1} = int32([]);
        caselut{4,2} = int32([]);
        caselut{4,3} = int32([]);
        caselut{4,4} = int32([]);
        caselut{4,5} = int32([]);
        caselut{4,6} = int32([]);
        caselut{4,7} = int32([]);
        caselut{4,8} = int32([]);
        caselut{4,9} = int32([]);
        caselut{4,10} = int32([]);
        caselut{4,11} = int32([]);
        caselut{4,12} = int32([]);
        caselut{4,13} = int32([]);
        caselut{4,14} = int32([]);
        caselut{4,15} = int32([]);
        caselut{4,16} = int32([]);
        caselut{4,17} = int32([]);
        caselut{4,18} = int32([]);
        caselut{4,19} = int32([]);
        caselut{4,20} = int32([]);
        caselut{4,21} = int32([]);
        caselut{4,22} = int32([]);
        caselut{4,23} = int32([]);
        caselut{4,24} = int32([]);
        caselut{4,25} = int32([]);
        caselut{4,26} = int32([]);
        caselut{4,27} = int32([]);
        caselut{4,28} = int32([]);
        caselut{4,29} = int32([]);
        caselut{4,30} = int32([]);
        caselut{4,31} = int32([]);
        caselut{4,32} = int32([]);
        caselut{5,1} = int32([]);
        caselut{5,2} = int32([]);
        caselut{5,3} = int32([]);
        caselut{5,4} = int32([]);
        caselut{5,5} = int32([]);
        caselut{5,6} = int32([]);
        caselut{5,7} = int32([]);
        caselut{5,8} = int32([]);
        caselut{5,9} = int32([]);
        caselut{5,10} = int32([]);
        caselut{5,11} = int32([]);
        caselut{5,12} = int32([]);
        caselut{5,13} = int32([]);
        caselut{5,14} = int32([]);
        caselut{5,15} = int32([]);
        caselut{5,16} = int32([]);
        caselut{5,17} = int32([]);
        caselut{5,18} = int32([]);
        caselut{5,19} = int32([]);
        caselut{5,20} = int32([]);
        caselut{5,21} = int32([]);
        caselut{5,22} = int32([]);
        caselut{5,23} = int32([]);
        caselut{5,24} = int32([]);
        caselut{5,25} = int32([]);
        caselut{5,26} = int32([]);
        caselut{5,27} = int32([]);
        caselut{5,28} = int32([]);
        caselut{5,29} = int32([]);
        caselut{5,30} = int32([]);
        caselut{5,31} = int32([]);
        caselut{5,32} = int32([]);
        caselut{6,1} = int32([]);
        caselut{6,2} = int32([]);
        caselut{6,3} = int32([]);
        caselut{6,4} = int32([]);
        caselut{6,5} = int32([]);
        caselut{6,6} = int32([]);
        caselut{6,7} = int32([]);
        caselut{6,8} = int32([]);
        caselut{6,9} = int32([]);
        caselut{6,10} = int32([]);
        caselut{6,11} = int32([]);
        caselut{6,12} = int32([]);
        caselut{6,13} = int32([]);
        caselut{6,14} = int32([]);
        caselut{6,15} = int32([]);
        caselut{6,16} = int32([]);
        caselut{6,17} = int32([]);
        caselut{6,18} = int32([]);
        caselut{6,19} = int32([]);
        caselut{6,20} = int32([]);
        caselut{6,21} = int32([]);
        caselut{6,22} = int32([]);
        caselut{6,23} = int32([]);
        caselut{6,24} = int32([]);
        caselut{6,25} = int32([]);
        caselut{6,26} = int32([]);
        caselut{6,27} = int32([]);
        caselut{6,28} = int32([]);
        caselut{6,29} = int32([]);
        caselut{6,30} = int32([]);
        caselut{6,31} = int32([]);
        caselut{6,32} = int32([]);
        caselut{7,1} = int32([]);
        caselut{7,2} = int32([]);
        caselut{7,3} = int32([]);
        caselut{7,4} = int32([]);
        caselut{7,5} = int32([]);
        caselut{7,6} = int32([]);
        caselut{7,7} = int32([]);
        caselut{7,8} = int32([]);
        caselut{7,9} = int32([]);
        caselut{7,10} = int32([]);
        caselut{7,11} = int32([]);
        caselut{7,12} = int32([]);
        caselut{7,13} = int32([]);
        caselut{7,14} = int32([]);
        caselut{7,15} = int32([]);
        caselut{7,16} = int32([]);
        caselut{7,17} = int32([]);
        caselut{7,18} = int32([]);
        caselut{7,19} = int32([]);
        caselut{7,20} = int32([]);
        caselut{7,21} = int32([]);
        caselut{7,22} = int32([]);
        caselut{7,23} = int32([]);
        caselut{7,24} = int32([]);
        caselut{7,25} = int32([]);
        caselut{7,26} = int32([]);
        caselut{7,27} = int32([]);
        caselut{7,28} = int32([]);
        caselut{7,29} = int32([]);
        caselut{7,30} = int32([]);
        caselut{7,31} = int32([]);
        caselut{7,32} = int32([]);
    end
 
    if (b_sync_V == true)
        cases = caselut{n,cond};
    else
        cases = int32([]);
    end
end