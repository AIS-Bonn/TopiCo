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

cd(fileparts(mfilename('fullpath')));
addpath(genpath(pwd));

disable_fprintf(false); % if true, fprintf is disabled to improve performance

for index_code_gen = 2

    if (index_code_gen == 1)
        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.EnableJIT = false;
        cfg.EnableOpenMP = false;
        cfg.GenCodeOnly = false;
        cfg.ResponsivenessChecks = true;
        cfg.SaturateOnIntegerOverflow = false;
        cfg.IntegrityChecks = true;
        cfg.EnableMexProfiling = true;
        cfg.ExtrinsicCalls = true;
    end

    if (index_code_gen == 2)
        cfg = coder.config('lib','ecoder',false);
        cfg.BuildConfiguration = 'Faster Runs';
        cfg.GenerateReport = false;
        cfg.RuntimeChecks = false;
        cfg.EnableOpenMP = false;
        cfg.GenCodeOnly = true;
        cfg.SaturateOnIntegerOverflow = false;
        cfg.SupportNonFinite = true;
        cfg.TargetLang = 'C++';
        cfg.TargetLangStandard = 'C++03 (ISO)';
        cfg.HardwareImplementation.ProdHWDeviceType = 'Intel->x86-64 (Linux 64)';
        cfg.HardwareImplementation.TargetHWDeviceType = 'Intel->x86-64 (Linux 64)';
    end

    ARGS = cell(1,1);
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(0,[Inf 3],[0 0]);         % State_start
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(0,[Inf 5 Inf],[0 0 0]);   % Waypoints
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(0,[Inf Inf],[0 0]);       % V_max
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(0,[Inf Inf],[0 0]);       % V_min
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(0,[Inf Inf],[0 0]);       % A_max
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(0,[Inf Inf],[0 0]);       % A_min
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(0,[Inf Inf],[0 0]);       % J_max
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(0,[Inf Inf],[0 0]);       % J_min
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(0,[Inf 1],[0 0]);         % A_global
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(false,[Inf Inf],[0 0]);   % b_sync_V
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(false,[Inf Inf],[0 0]);   % b_sync_A
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(false,[Inf Inf],[0 0]);   % b_sync_J
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(false,[Inf Inf],[0 0]);   % b_sync_W
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(false,[Inf Inf],[0 0]);   % b_rotate
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(false,[Inf Inf],[0 0]);   % b_hard_V_lim
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(false,[Inf Inf],[0 0]);   % b_catch_up
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(int8(0),[Inf Inf],[0 0]); % direction
    ARGS{1}{size(ARGS{1},2)+1} = coder.typeof(0,[1 1],[0 0]);           % ts_rollout
    
    if (index_code_gen == 1)
        codegen -v -config cfg topico -o topico_mex -args ARGS{1}
        copyfile(['codegen/mex/topico/topico_mex.',mexext],['topico_mex.',mexext]);
    elseif (index_code_gen == 2)
        codegen -v -config cfg topico_wrapper -args ARGS{1}
    end

end

