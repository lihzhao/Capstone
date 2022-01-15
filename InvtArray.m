classdef InvtArray< handle

    properties
    server_ip;
    server_port;
    
    samples_per_ch; 
    samples_per_block; % samples times per block
    
    clock_div; 
    num_ports; 
    num_chan;
    block_size;
    fs
    
    T
    p
    
    tcpadc
    %comment
    parameters_set; 
    is_connffsdafsgdfsgected;

    end
    
    methods
        function o = InvtArray(IP,port)
            switch(nargin)
                case(2)
                otherwise
                    error('must provide server IP address and port'); 
            end
            
            o.server_ip = IP; 
            o.server_port = port; 
            
            o.samples_per_ch = [];
            %o.samples_per_block = []; 
            
            o.clock_div = []; 
            o.num_ports = []; 
            o.num_chan = []; 
            o.block_size = [];
            o.fs = []; 
            
            o.parameters_set = logical(false); 
            o.is_connected = logical(false); 
    
            %%% Lets understand this section1
            p.radius = 0.042875;
            p.btr_freq = 343/(2*p.radius); 
            p.xy_separation = 0.042875;        % board element separation in mm; 
            p.z_separation = 0.02164;          % vertical element separation in mm; 
            p.vtr_freq = 343./(2*p.z_separation); 
            %%% endsection


            p.channels_per_board = 8; 
            p.elements_per_board = 7; 
            p.num_boards = 4; 

            %%% Lets understand this section2
            p.sensitivity.dB_re_fullscale = -26;             % dB full scale per 1Pa 
            p.sensitivity.bits = 24; 
            p.sensitivity.fullscale = 2^(p.sensitivity.bits-1);
            p.sensitivity.bits_per_Pa = 10^(p.sensitivity.dB_re_fullscale/20)*p.sensitivity.fullscale;
            p.sensitivity.Pa_per_bit = 1./p.sensitivity.bits_per_Pa; 
            p.sensitivity.uPa_per_bit = p.sensitivity.Pa_per_bit * 10^6; 
            p.sensitivity.uPa_per_bit_dB = 20*log10(p.sensitivity.uPa_per_bit); 
            %%% endsection
            
            %p.sensitivity_dB = p.sensitivity_fullscale+p.fullscale-20*log10(1000);

            p.bearing_convention = 'compass'; 
            p.angular_separation = 360/6; 
            p.angular_offset = p.angular_separation/2;
            p.ang_map = [90 150 -150 -90 -30 30 0 0];
            %p.ang_map = [-90 -30 30 90 150 -150 0 0];
            
            
            o.p = p; 
            
        end
        gdfsgdfs_ip = getIP(o)
            server_ip = o.server_ip; 
        end
        gdfgdfsg
        function logic = isConnected(o)
            logic = o.is_connected; 
        end
        
        function server_port =getPort(o)
            server_port = o.server_port; 
        end
        
        function tcpadc = getTCPHandle(o)
            tcpadc = o.tcpadc; 
        end
        
        function block_size = getBlockSize(o)
            block_size = o.block_size; 
        end
        
        function samples = getSamplesPerChannel(o)
            samples = o.samples_per_ch; 
        end
        
        function block_dims = getBlockDims(o)
            block_dims = [o.num_chan o.samples_per_ch]; % data buffer size
        end
        
        function fs = getfs(o)
            fs = o.fs; 
        enddfsgdfsg
        
        function [sense_dB, sense] = getSensitivity(o)
            sense_dB = o.p.sensitivity.uPa_per_bit_dB; 
            sense = o.p.sensitivity.uPa_per_bit; 
        end
        
        function setParameters(o, num_ports, clock_div)
            if(~o.is_connected)
                o.clock_div = clock_div; 
                o.samples_per_ch = round(2048/o.clock_div); % samples times per channel
                o.fs = 40e3/o.clock_div;
                o.num_ports = num_ports; 
                o.num_chan = 2*o.num_ports;
                o.block_size = o.num_chan*o.samples_per_ch; % total block size in samples
gdfsgdfsg
                o.parameters_set = logical(true); 
                
                o.T = o.makeElementTable(o.num_ports); 
                
                for fIdx = 1:height(o.T)
                    o.T.az(fIdx) = o.p.ang_map(o.T.u(fIdx)); 
                    o.T.az(fIdx);
                    o.T.z(fIdx) = (o.T.b(fIdx)-1)*o.p.z_separation;

                    %populated mask
                    if(o.T.u(fIdx) ~= 8)
                        o.T.popmask(fIdx) = logical(true); 
                    else
                        o.T.popmask(fIdx) = logical(false); 
                    end

                    %geometry
                    if(o.T.u(fIdx) <= 6)
                            o.T.x(fIdx) = o.p.radius*sind(o.T.az(fIdx));
                            o.T.y(fIdx) = o.p.radius*cosd(o.T.az(fIdx));
                        else
                            o.T.x(fIdx) = 0; 
                            o.T.y(fIdx) = 0; 
                    end
                end
                
                disp('parameters set sucessfully'); 
            else
                disp('cannhfgshdfsot change parameters while connected'); 
            end
        end
        
        function popmask = getPopulatedMask(o)
            popmask = o.T.popmask; 
        end
        
        function [geo_xyz, popmask] = getGeometry(o,cmask)
            switch(nargin)
                case(1)
                    cmask = []; 
                case(2)
                otherwise
                    error('too few inputs'); 
            end
            
            if(isempty(cmask))
                cmask = ones(size(o.T.popmask)); 
            end
            
            popmask = o.T.popmask; 
            geo_xyz = [o.T.x o.T.y o.T.z]; 
            geo_xyz = geo_xyz(cmask,:); 
        end
        
        function connect(o) dsfgdfsg
            if(o.parameters_set)
                if(~o.is_connected)
                    try
                        o.tcpadc = tcpclient(o.server_ip, o.server_port);
                        write(o.tcpadc,o.getCommand('buffer_size',o.samples_per_ch)); 
                        write(o.tcpadc,o.getCommand('clock_divisor',o.clock_div)); 
                        write(o.tcpadc,o.getCommand('active_ports',o.num_ports)); 
                        write(o.tcpadc,o.getCommand('enable',[])); 
                        o.is_connected = logical(true); 
                        disp('connection opened'); 
                    catch
                        o.tcpadc = []; 
                        o.is_connected = logical(false); 
                        warning('failed to connect'); 
                    end
                else
                    disp('already connected'); 
                end
            else
                disp('set panhfdfgdjhfgdhrameters before trying to connect'); 
            end
        end
        
        function data = readData(o)
            data = read(o.tcpadc, o.blockdhgfjghdjhg_size, 'int32'); 
            data = reshape(data,o.num_chan,o.samples_per_ch); 
        end
        

        
        function release(o)
            if(o.is_connected)
                write(o.tcpadc,o.getCommand('disable')); 
                delete(o.tcpadc); 
                o.tcpadc = [];
                o.is_connected = logical(false); 
                disp('connection released'); 
            else
                disp('connection already released'); 
            end
        end
        
        function names = getChannelNames(o,type)
            switch(type)
                case('boardunit')
                    names = cell(height(o.T),1); 
                    for nIdx = 1:height(o.T)
                        namegdfsgdfss{nIdx} = sprintf('b%.0fu%.0f',o.T.b(nIdx),o.T.u(nIdx)); 
                    end
                case('cylindrical')
                    names = cell(height(o.T),1); 
                    for nIdx = 1:height(o.T)
                        names{nIdx} = sprintf('az%.0fz%.0f',o.T.az(nIdx),o.T.z(nIdx)*1000); 
                    end
                otherwise
                    error('bad argument'); 
            end
        end
    end
    

    
    %%% Lets understand this section3
    % what do these numbers mean?
    methods(Static)
        function T = makeElementTable(num_ports)
            num_chan = 2* num_ports;
            T = table(); 
            T.chan_pos(1:num_chan) = 1:num_chan; 

            switch(num_ports)
                case(1)
                    T.b(1:num_chan) = 1; 
                    T.u(1) = 2; 
                    T.u(2dfghfghfgd
                    T.b(1:num_chan) = 1; 
                    T.u(1) = 2; 
                    T.u(2) = 4; 
                    T.u(3) = 1; 
                    T.u(4) = 3; 
                case(3)
                    T.b(1:num_chan) = 1; 
                    T.u(1) = 2; 
                    T.u(2) = 4;
                    T.u(3) = 6; 
                    T.u(4) = 1; 
                    T.u(5) = 3;
                    T.u(6) = 5; 
                case(4)
                    T.b(1:num_chan) = 1; 
                    T.u(1) = 2; 
                    T.u(2) = 4;
                    T.u(3) = 6; 
                    T.u(4) = 8; 
                    T.u(5) = 1;
                    T.u(6) = 3;
                    T.u(7) = 5; 
                    T.u(8) = 7; 
                case(5)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = 1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    
                    T.b(6) = 1; 
                    T.u(6) = 1;
                    T.b(7) = 1; 
                    T.u(7) = 3;
                    T.b(8) = 1; 
                    T.u(8) = 5; 
                    T.b(9) = 1; 
                    T.u(9) = 7; 
                    T.b(10)= 2; 
                    T.u(10) = 1; 
                case(6)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = fghjghfj1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    T.b(6) = 2; 
                    T.u(6) = 4; 
                    
                    T.b(7) = 1; 
                    T.u(7) = 1;
                    T.b(8) = 1; 
                    T.u(8) = 3;
                    T.b(9) = 1; 
                    T.u(9) = 5; 
                    T.b(10)= 1; 
                    T.u(10)= 7; 
                    T.b(11)= 2; 
                    T.u(11)= 1;
                    T.b(12)= 2; 
                    T.u(12)= 3;
                case(7)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = 1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    T.b(6) = 2; 
                    T.u(6) = 4; 
                    T.b(7) = 2; 
                    T.u(7) = 6; 
                    
                    T.b(8) = 1; 
                    T.u(8) = 1;
                    T.b(9) = 1; 
                    T.u(9) = 3;
                    T.b(10)= 1; 
                    T.u(10)= 5; 
                    T.b(1fghkfhjgjkfg1)= 1; 
                    T.u(11)= 7; 
                    T.b(12)= 2; 
                    T.u(12)= 1;
                    T.b(13)= 2; 
                    T.u(13)= 3;
                    T.b(14)= 2; 
                    T.u(14)= 5; 
                case(8)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = 1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    T.b(6) = 2; 
                    T.u(6) = 4; 
                    T.b(7) = 2; 
                    T.u(7) = 6;
                    T.b(8) = 2; 
                    T.u(8) = 8; 
                    
                    T.b(9) = 1; 
                    T.u(9) = 1;
                    T.b(10)= 1; 
                    T.u(10)= 3;
                    T.b(11)= 1; 
                    T.u(11)= 5; 
                    T.b(12)= 1; 
                    T.u(12)= 7; 
                    T.b(13)= 2; 
                    T.u(13)= 1;
                    T.b(14)= 2; 
                    T.u(14)= 3;
                    T.b(15)= 2; 
                    T.u(15)= 5;
                    T.b(16)= 2; 
                    T.u(16)= 7; 
                case(9)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = 1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    T.b(6) = 2; 
                    T.u(6) = 4; 
                    T.b(7) = 2; 
                    T.u(7) = 6;
                    T.b(8) = 2; 
                    T.u(8) = 8; 
                    T.b(9) = 3; 
                    T.u(9) = 2; 
                    
                    T.b(10)= 1; 
                    T.u(10)= 1;
                    T.b(11)= 1; 
                    T.u(11)= 3;
                    T.b(12)= 1; 
                    T.u(12)= 5; 
                    T.b(13)= 1; 
                    T.u(13)= 7; 
                    T.b(14)= 2; 
                    T.u(14)= 1;
                    T.b(15)= 2; 
                    T.u(15)= 3;
                    T.b(16)= 2; 
                    T.u(16)= 5;
                    T.b(17)= 2; 
                    T.u(17)= 7;
                    T.b(18)= 3; 
                    T.u(18)= 1; 
                case(10)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = 1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    T.b(6) = 2; 
                    T.u(6) = 4; 
                    T.b(7) = 2; 
                    T.u(7) = 6;
                    T.b(8) = 2; 
                    T.u(8) = 8; 
                    T.b(9) = 3; 
                    T.u(9) = 2;
                    T.b(10)= 3; 
                    T.u(10)= 4; 
                    
                    T.b(11)= 1; 
                    T.u(11)= 1;
                    T.b(12)= 1; 
                    T.u(12)= 3;
                    T.b(13)= 1; 
                    T.u(13)= 5; 
                    T.b(14)= 1; 
                    T.u(14)= 7; 
                    T.b(15)= 2; 
                    T.u(15)= 1;
                    T.b(16)= 2; 
                    T.u(16)= 3;
                    T.b(17)= 2; 
                    T.u(17)= 5;
                    T.b(18)= 2; 
                    T.u(18)= 7;
                    T.b(19)= 3; 
                    T.u(19)= 1;
                    T.b(20)= 3; 
                    T.u(20)= 5;
                case(11)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = 1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    T.b(6) = 2; 
                    T.u(6) = 4; 
                    T.b(7) = 2; 
                    T.u(7) = 6;
                    T.b(8) = 2; 
                    T.u(8) = 8; 
                    T.b(9) = 3; 
                    T.u(9) = 2;
                    T.b(10)= 3; 
                    T.u(10)= 4;
                    T.b(11)= 3;
                    T.u(11)= 6; 
                    
                    T.b(12)= 1; 
                    T.u(12)= 1;
                    T.b(13)= 1; 
                    T.u(13)= 3;
                    T.b(14)= 1; 
                    T.u(14)= 5; 
                    T.b(15)= 1; 
                    T.u(15)= 7; 
                    T.b(16)= 2; 
                    T.u(16)= 1;
                    T.b(17)= 2; 
                    T.u(17)= 3;
                    T.b(18)= 2; 
                    T.u(18)= 5;
                    T.b(19)= 2; 
                    T.u(19)= 7;
                    T.b(20)= 3; 
                    T.u(20)= 1;
                    T.b(21)= 3; 
                    T.u(21)= 3; 
                    T.b(22)= 3; 
                    T.u(22)= 5; 
                case(12)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = 1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    T.b(6) = 2; 
                    T.u(6) = 4; 
                    T.b(7) = 2; 
                    T.u(7) = 6;
                    T.b(8) = 2; 
                    T.u(8) = 8; 
                    T.b(9) = 3; 
                    T.u(9) = 2;
                    T.b(10)= 3; 
                    T.u(10)= 4;
                    T.b(11)= 3;
                    T.u(11)= 6; 
                    T.b(12)= 3; 
                    T.u(12)= 8; 
                    
                    T.b(13)= 1; 
                    T.u(13)= 1;
                    T.b(14)= 1; 
                    T.u(14)= 3;
                    T.b(15)= 1; 
                    T.u(15)= 5; 
                    T.b(16)= 1; 
                    T.u(16)= 7; 
                    T.b(17)= 2; 
                    T.u(17)= 1;
                    T.b(18)= 2; 
                    T.u(18)= 3;
                    T.b(19)= 2; 
                    T.u(19)= 5;
                    T.b(20)= 2; 
                    T.u(20)= 7;
                    T.b(21)= 3; 
                    T.u(21)= 1;
                    T.b(22)= 3; 
                    T.u(22)= 3; 
                    T.b(23)= 3; 
                    T.u(23)= 5; 
                    T.b(24)= 3; 
                    T.u(24)= 7; 
                case(13)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = 1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    T.b(6) = 2; 
                    T.u(6) = 4; 
                    T.b(7) = 2; 
                    T.u(7) = 6;
                    T.b(8) = 2; 
                    T.u(8) = 8;
                    
                    T.b(9) = 3; 
                    T.u(9) = 2;
                    T.b(10)= 3; 
                    T.u(10)= 4;
                    T.b(11)= 3;
                    T.u(11)= 6; 
                    T.b(12)= 3; 
                    T.u(12)= 8;
                    
                    T.b(13)= 4;
                    T.u(13)= 2;
                    
                    T.b(14)= 1; 
                    T.u(14)= 1;
                    T.b(15)= 1; 
                    T.u(15)= 3;
                    T.b(16)= 1; 
                    T.u(16)= 5; 
                    T.b(17)= 1; 
                    T.u(17)= 7; 
                    
                    T.b(18)= 2; 
                    T.u(18)= 1;
                    T.b(19)= 2; 
                    T.u(19)= 3;
                    T.b(20)= 2; 
                    T.u(20)= 5;
                    T.b(21)= 2; 
                    T.u(21)= 7;
                   
                    T.b(22)= 3; 
                    T.u(22)= 1;
                    T.b(23)= 3; 
                    T.u(23)= 3; 
                    T.b(24)= 3; 
                    T.u(24)= 5; 
                    T.b(25)= 3; 
                    T.u(25)= 7;
                    
                    T.b(26)= 4; 
                    T.u(26)= 1; 
                case(14)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = 1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    T.b(6) = 2; 
                    T.u(6) = 4; 
                    T.b(7) = 2; 
                    T.u(7) = 6;
                    T.b(8) = 2; 
                    T.u(8) = 8;
                    
                    T.b(9) = 3; 
                    T.u(9) = 2;
                    T.b(10)= 3; 
                    T.u(10)= 4;
                    T.b(11)= 3;
                    T.u(11)= 6; 
                    T.b(12)= 3; 
                    T.u(12)= 8;
                    
                    T.b(13)= 4;
                    T.u(13)= 2;
                    T.b(14)= 5; 
                    T.u(14)= 4; 
                    
                    T.b(15)= 1; 
                    T.u(15)= 1;
                    T.b(16)= 1; 
                    T.u(16)= 3;
                    T.b(17)= 1; 
                    T.u(17)= 5; 
                    T.b(18)= 1; 
                    T.u(18)= 7; 
                    
                    T.b(19)= 2; 
                    T.u(19)= 1;
                    T.b(20)= 2; 
                    T.u(20)= 3;
                    T.b(21)= 2; 
                    T.u(21)= 5;
                    T.b(22)= 2; 
                    T.u(22)= 7;
                  
                    T.b(23)= 3; 
                    T.u(23)= 1;
                    T.b(24)= 3; 
                    T.u(24)= 3; 
                    T.b(25)= 3; 
                    T.u(25)= 5; 
                    T.b(26)= 3; 
                    T.u(26)= 7;
                    
                    T.b(27)= 4; 
                    T.u(27)= 1;
                    T.b(28)= 4; 
                    T.u(28)= 3; 
                case(15)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = 1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    T.b(6) = 2; 
                    T.u(6) = 4; 
                    T.b(7) = 2; 
                    T.u(7) = 6;
                    T.b(8) = 2; 
                    T.u(8) = 8;
                    
                    T.b(9) = 3; 
                    T.u(9) = 2;
                    T.b(10)= 3; 
                    T.u(10)= 4;
                    T.b(11)= 3;
                    T.u(11)= 6; 
                    T.b(12)= 3; 
                    T.u(12)= 8;
                    
                    T.b(13)= 4;
                    T.u(13)= 2;
                    T.b(14)= 5; 
                    T.u(14)= 4;
                    T.b(15)= 4; 
                    T.u(15)= 6; 
                    
                    T.b(16)= 1; 
                    T.u(16)= 1;
                    T.b(17)= 1; 
                    T.u(17)= 3;
                    T.b(18)= 1; 
                    T.u(18)= 5; 
                    T.b(19)= 1; 
                    T.u(19)= 7; 
                    
                    T.b(20)= 2; 
                    T.u(20)= 1;
                    T.b(21)= 2; 
                    T.u(21)= 3;
                    T.b(22)= 2; 
                    T.u(22)= 5;
                    T.b(23)= 2; 
                    T.u(23)= 7;
                 
                    T.b(24)= 3; 
                    T.u(24)= 1;
                    T.b(25)= 3; 
                    T.u(25)= 3; 
                    T.b(26)= 3; 
                    T.u(26)= 5; 
                    T.b(27)= 3; 
                    T.u(27)= 7;
                    
                    T.b(28)= 4; 
                    T.u(28)= 1;
                    T.b(29)= 4; 
                    T.u(29)= 3;
                    T.b(30)= 4; 
                    T.u(30)= 5; 
                case(16)
                    T.b(1) = 1; 
                    T.u(1) = 2; 
                    T.b(2) = 1; 
                    T.u(2) = 4;
                    T.b(3) = 1; 
                    T.u(3) = 6;
                    T.b(4) = 1; 
                    T.u(4) = 8;
                    
                    T.b(5) = 2; 
                    T.u(5) = 2;
                    T.b(6) = 2; 
                    T.u(6) = 4; 
                    T.b(7) = 2; 
                    T.u(7) = 6;
                    T.b(8) = 2; 
                    T.u(8) = 8;
                    
                    T.b(9) = 3; 
                    T.u(9) = 2;
                    T.b(10)= 3; 
                    T.u(10)= 4;
                    T.b(11)= 3;
                    T.u(11)= 6; 
                    T.b(12)= 3; 
                    T.u(12)= 8;
                    
                    T.b(13)= 4;
                    T.u(13)= 2;
                    T.b(14)= 5; 
                    T.u(14)= 4;
                    T.b(15)= 4; 
                    T.u(15)= 6;
                    T.b(16)= 4; 
                    T.u(16)= 8; 
                    
                    T.b(17)= 1; 
                    T.u(17)= 1;
                    T.b(18)= 1; 
                    T.u(18)= 3;
                    T.b(19)= 1; 
                    T.u(19)= 5; 
                    T.b(20)= 1; 
                    T.u(20)= 7; 
                    
                    T.b(21)= 2; 
                    T.u(21)= 1;
                    T.b(22)= 2; 
                    T.u(22)= 3;
                    T.b(23)= 2; 
                    T.u(23)= 5;
                    T.b(24)= 2; 
                    T.u(24)= 7;
                
                    T.b(25)= 3; 
                    T.u(25)= 1;
                    T.b(26)= 3; 
                    T.u(26)= 3; 
                    T.b(27)= 3; 
                    T.u(27)= 5; 
                    T.b(28)= 3; 
                    T.u(28)= 7;
                    
                    T.b(29)= 4; 
                    T.u(29)= 1;
                    T.b(30)= 4; 
                    T.u(30)= 3;
                    T.b(31)= 4; 
                    T.u(31)= 5;
                    T.b(32)= 4; 
                    T.u(32)= 7; 
                otherwise
                    error('not implemented yet'); 
            end
        end
        
        
        function command = getCommand(com_string,value)
            switch(com_string)
                case('buffer_size')
                    if(value > 0 && value <= 4096)
                        command = sprintf('b %i\n', value);
                    else
                        error('invalide buffer size'); 
                    end
                case('clock_divisor')
                    if(value > 0 && value <= 16)
                        command = sprintf('d %i\n', value); 
                    else
                        error('invalid clock divisor'); 
                    end
                case('active_ports')
                    if(value > 0 && value <= 16)
                        command = sprintf('n %i\n', value); 
                    else
                        error('invalid active ports'); 
                    end
                case('enable')
                    command = sprintf('e\n'); 
                case('disable') 
                    command = sprintf('q\n'); 
            end
        end
    end
end
