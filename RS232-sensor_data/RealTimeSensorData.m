%
% RC car serial port utility
%
% Provides real-time plots of accelerometer and gyrometer sensor data.
%
% TODO:
%    - use preallocated memory and var(:) assignments in tight sections

%% main interface fcn
function [Aa,Ag] = RealTimeSensorData(varargin)
    format compact;
    
    % DISABLE these for release
    %%{
    clear global Aa;
    global Aa;
    clear global Ag;
    global Ag;
    clear global Cxg;
    global Cxg;
    clear global Cyg;
    global Cyg;
    clear global Czg;
    global Czg;
    clear global a;
    global a;
    clear global g;
    global g;
    clear global Abias;
    global Abias;
    clear global Gbias;
    global Gbias;
    %%}

    % NOTE: when deciding on params here, to make the plots smoothly flow
    % off the screen, all in sync, need to consider:
    %   1. total_depth_time / update_rate is integer
    %   2. update_rate / 24ms is integer
    %   3. total_depth_time / 24ms is integer
    
    % 96ms update (4*24ms), 5.088sec total depth (212*24ms), 5.088/0.096=53
    update_rate = 0.096;
    total_depth_time = 5.088;
    
    % 192ms update (8*24ms), 10.176sec total depth (424*24ms), 10.123/0.192=53
    %update_rate = 0.192;
    %total_depth_time = 10.176;
    
    if isequal(nargin,1)
        update_rate=varargin{1};
    elseif isequal(nargin,2)
        update_rate = varargin{1};
        total_depth_time = varargin{2};
    end
    
    Aa = [];
    Ag = [];
    Cxg = [];
    Cyg = [];
    Czg = [];
    a = [];
    g = [];
    Abias = 0;
    Gbias = 0;

    %   'bytes' - number of bytes per ReadSerial
    %   'scans' - number of accel/gyro/etc samples total in the plot window
    %             history
    %   'scansA' - accel samples in plot
    %   'scansG' - gyro samples  in plot
    bytes = update_rate * 18 / 0.024;
    scans = total_depth_time / 0.008;
    scansA = uint32(scans);
    %scansG = uint32(total_depth_time/0.012);
    scansG = scansA;    % b/c we now pad G buffer so time sync in maintained
    scansCxg = uint32(total_depth_time/update_rate);
    scansCyg = uint32(total_depth_time/update_rate);
    scansCzg = uint32(total_depth_time/update_rate);
    
    figureInit;
    
    % because of break characters in serial stream, we add a few extra
    % bytes to ReadSerial buffer. this should ensure we get the correct
    % number of A&G samples per read.
    % TODO: actually, the extra bytes are dependent on the number of bytes
    % being read, for 72bytes(96ms), add in 2 as we will at most only see 2
    % break characters.
    s = OpenSerial('/dev/ttyUSB0',115200,bytes+2);  % 21cycles@24ms of 18bytes each = 0.504 seconds of data
    
    %loopcnt = 0;
    
    while true
        
      % read in a buffer of serial data
      [buf,err2] = ReadSerial(s);
      if ~isequal(err2,0)
          errordlg('ERROR:RealTimeSensorData:ReadSerial returned error!');
          continue;
      end
      
      % process buffer of data, if good, append to our running log
      [a,g,err] = ProcessBuffer(buf);
      if ~isequal(err,0)
          errordlg('FATAL: ProcessBuffer returned error');
          continue; % just keep going...
      end
      
      % accel
      Aa = [Aa;a];
      sizeA = size(Aa,1)+1 - scansA;
      if sizeA < 1
          sizeA = 1;
      end
      Aa = Aa(sizeA:end,:);
      
      % gyro
      Ag = [Ag;g];
      sizeG = size(Ag,1)+1 - scansG;
      if sizeG < 1
          sizeG = 1;
      end
      Ag = Ag(sizeG:end,:);
      
      % update cov stats
      if isempty(Cxg)
          disp('ACCEL');
        Abias = sum(a)/size(a,1);
        Cxg = [sum(a)-Abias*size(a,1)];
      else
          %{
        disp(sprintf('ACCEL:sum=%d,%d,%d Abias=%d,%d,%d size(a)=%d next=%d,%d,%d\n',...
            sum(a(:,1)),sum(a(:,2)),sum(a(:,3)),...
            Abias(1),Abias(2),Abias(3),...
            size(a,1),...
            Cxg(:,1)+sum(a(:,1))-Abias(1)*size(a,1),...
            Cxg(:,2)+sum(a(:,2))-Abias(2)*size(a,1),...
            Cxg(:,3)+sum(a(:,3))-Abias(3)*size(a,1)));
          %}
        Cxg = [Cxg;Cxg(end,:)+sum(a)-Abias*size(a,1)];
      end
      sizeCxg = size(Cxg,1)+1 - scansCxg;
      if sizeCxg < 1
        sizeCxg = 1;
      end
      Cxg = Cxg(sizeCxg:end,:);

      if isempty(Cyg)
          disp('GYRO');
        Gbias = sum(g)/size(g,1);
        Cyg = [sum(g)-Gbias*size(g,1)];
      else
          %{
        disp(sprintf('GYRO:sum=%d Gbias=%d size(g)=%d next=%d\n',...
            sum(g),Gbias,f,Cyg(end)+sum(g)-Gbias*size(g,1)));
          %}
        Cyg = [Cyg;Cyg(end,:)+sum(g)-Gbias*size(g,1)];
      end
      sizeCyg = size(Cyg,1)+1 - scansCyg;
      if sizeCyg < 1
        sizeCyg = 1;
      end
      Cyg = Cyg(sizeCyg:end,:);
          
      %{
      if ~isempty(a)
          if size(a,1) >= size(g,1)
            cxgx = cov([a(1:size(g,1),1) g]);
            Cxg = [Cxg;cxgx(2,2)];
          else
            if isempty(Cxg)
                warndlg('WARNING: check this: Cxg isempty = [0]');
                Cxg = [0];
            else
                Cxg = [Cxg;Cxg(end)];
            end
          end
          sizeCxg = size(Cxg,1)+1 - scansCxg;
          if sizeCxg < 1
          	sizeCxg = 1;
          end
          Cxg = Cxg(sizeCxg:end,:);

          cygx = cov([a(:,1) a(:,3)]);
          Cyg = [Cyg;cygx(2,2)];
          sizeCyg = size(Cyg,1)+1 - scansCyg;
          if sizeCyg < 1
          	sizeCyg = 1;
          end
          Cyg = Cyg(sizeCyg:end,:);

          %czgx = cov([a(:,2) a(:,3)]);
          %Czg = [Czg;czgx(2,2)];
          Czg = [Czg;a(end,1)*a(end,2)];
          sizeCzg = size(Czg,1)+1 - scansCzg;
          if sizeCzg < 1
          	sizeCzg = 1;
          end
          Czg = Czg(sizeCzg:end,:);
      end
      %}
      
      %{
      loopcnt = loopcnt + 1;
      if loopcnt > 10
          loopcnt = 0;
          disp(sprintf('ANGLE:x/z=%.3f(ang=%.3f) y/z=%.3f(ang=%.3f)',...
              mean(a(:,1))/mean(a(:,3)),...
              atan(mean(a(:,1))/mean(a(:,3)))*360/(2*pi),...
              mean(a(:,2))/mean(a(:,3)),...
              atan(mean(a(:,2))/mean(a(:,3)))*360/(2*pi)));
      end
      %}
      
      PostProcess1(Aa,Ag,Cxg,Cyg,Czg);

    end
    
    CloseSerial(s);
end

%% does plot updates and computes corr
% New style with 8 subplots in two columns
function PostProcess0(Aa,Ag,Cxg,Cyg,Czg)
    % NOTE: adding legend info makes plot updates too slow
    mxplty = 4;
    mxpltx = 2;
    windowSZ = 10;  % apply a moving window avg to deriv plots
    
    ax(1) = subplot(mxplty,mxpltx,1);
    plot(Aa,'Parent',ax(1));
    %legend('X','Y','Z');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ax(2) = subplot(mxplty,mxpltx,2);
    plot(Ag,'Parent',ax(2));
    %legend('G');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ax(3) = subplot(mxplty,mxpltx,3);
    plot(filter(ones(1,windowSZ)/windowSZ,1,diff(Aa)),'Parent',ax(3));
    %legend('X','Y','Z');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ax(4) = subplot(mxplty,mxpltx,4);
    plot(filter(ones(1,windowSZ)/windowSZ,1,diff(Ag)),'Parent',ax(4));
    %legend('G');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    linkaxes(ax,'x');
    
    [h,j] = gradient(Aa);
    [n] = gradient(Ag);
          
    ay(1) = subplot(mxplty,mxpltx,5);
    contour(h,'Parent',ay(1));
    %legend('Cxg'); 
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ay(2) = subplot(mxplty,mxpltx,6);
    contour(j,'Parent',ay(2));
    %legend('Cyg');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ay(3) = subplot(mxplty,mxpltx,7);
    contour(n,'Parent',ay(3));
    %legend('Czg');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

    linkaxes(ay,'x');
  
    %{
    h = get(0,'CurrentFigure');
    %h = gcf;
    set(h,'BackingStore','off',...
        'DoubleBuffer','on');
    %}
    
    drawnow;
end

%% does plot updates and computes corr
% New style with 8 subplots in two columns
function PostProcess1(Aa,Ag,Cxg,Cyg,Czg)
    % NOTE: adding legend info makes plot updates too slow
    mxplty = 3;
    mxpltx = 2;
    windowSZ = 20;  % apply a moving window avg to deriv plots
   
    ax(1) = subplot(mxplty,mxpltx,1);
    %plot([Aa(1:size(Ag),1) Aa(1:size(Ag),2) Ag/max([abs(max(Aa(:,1)));abs(max(Aa(:,2)))])],'Parent',ax(1));
    plot(Aa,'Parent',ax(1));
    %legend('X','Y','Z');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ax(2) = subplot(mxplty,mxpltx,2);
    %plot(Aa(:,3),'Parent',ax(2));
    plot(Ag,'Parent',ax(2));
    %legend('G');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ax(3) = subplot(mxplty,mxpltx,3);
    plot(filter(ones(1,windowSZ)/windowSZ,1,diff(Aa)),'Parent',ax(3));
    %legend('X','Y','Z');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ax(4) = subplot(mxplty,mxpltx,4);
    plot(filter(ones(1,windowSZ)/windowSZ,1,diff(Ag)),'Parent',ax(4));
    %legend('G');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    linkaxes(ax,'x');
        
    ay(1) = subplot(mxplty,mxpltx,5);
    plot(Cxg,'Parent',ay(1));
    %legend('Cxg'); 
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ay(2) = subplot(mxplty,mxpltx,6);
    plot(Cyg,'Parent',ay(2));
    %legend('Cyg');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    linkaxes(ay,'x');
        
    %{
    ay(3) = subplot(mxplty,mxpltx,7);
    plot(Czg,'Parent',ay(3));
    %legend('Czg');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

    linkaxes(ay,'x');
    %}
    
    %{
    h = get(0,'CurrentFigure');
    %h = gcf;
    set(h,'BackingStore','off',...
        'DoubleBuffer','on');
    %}
    
    drawnow;
end

%% does plot updates and computes corr
% this one is the old 5 window figure window
function PostProcess2(Aa,Ag,Cxg,Cyg,Czg)
    % NOTE: adding legend info makes plot updates too slow
    mxplt = 5;
    
    ax(1) = subplot(mxplt,1,1);
    plot(Aa,'Parent',ax(1));
    %legend('X','Y','Z');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ax(2) = subplot(mxplt,1,2);
    plot(Ag,'Parent',ax(2));
    %legend('G');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

    linkaxes(ax,'x');
    
    ay(1) = subplot(mxplt,1,3);
    plot(Cxg,'Parent',ay(1));
    %legend('Cxg'); 
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ay(2) = subplot(mxplt,1,4);
    plot(Cyg,'Parent',ay(2));
    %legend('Cyg');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
    
    ay(3) = subplot(mxplt,1,5);
    plot(Czg,'Parent',ay(3));
    %legend('Czg');
    set(gca, 'Position', get(gca, 'OuterPosition') - ...
        get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

    linkaxes(ay,'x');
  
    %{
    h = get(0,'CurrentFigure');
    %h = gcf;
    set(h,'BackingStore','off',...
        'DoubleBuffer','on');
    %}
    
    drawnow;
end

%% get figure ready for plotting
function figureInit
    close all;
    %h = get(0,'CurrentFigure');
    h = gcf;
    set(h,'BackingStore','off',...
        'DoubleBuffer','on');
end

%% open serial socket
function [socket] = OpenSerial(port,rate,num)
    % FOR DEBUG - clears any open serial ports
    c=instrfind;
    if ~isequal(c,[])
        fclose(c);
    end
    delete(c);
    clear c;
    % FOR DEBUG - clears any open serial ports
    
    socket=serial(port,'BaudRate',rate,'InputBufferSize',num);
    fopen(socket);
end

%% read block from serial port
function [buf,err] = ReadSerial(socket)
    buf = [];
    err = 0;
    buf=fread(socket);
    if isempty(buf)
        err = 1;
        errordlg('ERROR:ReadSerial:fread returned a null buffer!');
        return;
    end
end

%% read block from serial port
function [A,G,err] = ProcessBuffer(buf)
    A=[];
    G=[];
    err = 0;
    resync = 1;
    
    if isempty(buf)
        errordlg('ERROR:ProcessBuffer called with empty buffer!');
        err = 1;
        return;
    end
    
    %disp('START:ProcessBuffer(buf)');
    
    % RESYNC should come back here
    % 'buf' contains the data we will process, ptr to good start
    % stored in 'b_start'
    %
    % TODO: this is bullsh*t - this should be recoded to use a sync_pattern as
    % done in Perl script pwatch.pl
    %
    while resync == 1
        %disp('ProcessBuffer:SYNC START');
        nsync = 0;
        b_start = buf;
        [a,b] = strtok(buf,'AG');   % TODO: need to replace with findstr()
        %disp(sprintf('SYNC:STRTOK:b_start=%d a=%d b=%d',b_start(1),a(1),b(1)));
        while size(b,1) > 0
            
            % try to find where the 'A' or 'G' label is in the original
            % b_start buffer so we know what type of data is in the 'a'
            % from strtok 
            b_start_indx = size(b_start,1) - size(b,1) - size(a,1);
            if b_start_indx < 1
                %----------------------------------------------------------
                % TODO: I don't understand why, but sometimes strtok will
                % place the correct sample into 'a' but 'b' will contain 
                % the 'A'+3bytes at the start:
                %   b_start = 255   251    63    65   255   251    63    71
                %   a = 255   251    63
                %   b = 65   255   251    63    71     0    10
                %   size(b_start,1) = 76
                %   size(a,1) = 3
                %   size(b,1) = 73
                % In this case, it appears strtok messed up, so instead of
                % making b_start_indx = 76-73-3, we correctly set
                % b_start_indx = 76-73 so we find the correct 'A' label in
                % b_start[3]...
                %
                % Here a gyro, but correct sample is not in 'a':
                %   b_start = 250    63    71     0    10    65   255   250
                %   a = 250    63
                %   b = 71     0    10    65   255   250    63  
                %   size(b_start,1) = 76
                %   size(a,1) = 2
                %   size(b,1) = 74
                % Here setting b_start_indx=76-74 but
                % b_start(2)=63 which is incorrect, skip this sample and
                % return to strtok
                %
                % If our b_start_indx guess is incorrect, we will resync
                % below
                %----------------------------------------------------------
                b_start_indx = size(b_start,1) - size(b,1);
            end
            
            if isequal(b_start(b_start_indx),'A')
                %% accel (first)
                %%===============
                    % size will be 3 bytes for X,Y,Z or 5 if there are two break
                    % characters in there also
                if isequal(size(a,1),3) || isequal(size(a,1),5)
                    if isequal(b(1),'G')
                        %% gyro (next)
                        %%==================
                        if size(b,1) >= 4
                            if isequal(b(4),'G') || isequal(b(4),'A')
                                %
                                % - first sample = accel w/ a=3||5 bytes
                                % - gyro next and the next sample after
                                %     that appears to be a gyro or accel
                                nsync = 1;
                                break
                            end
                        else
                            %
                            % - first=accel w/3||5 bytes
                            % - then G but appears we lost G sample, so sync
                            %      is ok so we process accel sample
                            nsync = 1;
                            break;
                        end
                    elseif isequal(b(1),'A')
                        %% accel (next)
                        %====================
                        if size(b,1) >= 5
                            if isequal(b(5),'G') || isequal(b(5),'A')
                                %
                                % - first sample = accel w/ a=3||5 bytes
                                % - accel next and the next sample after
                                %     that appears to be a gyro or accel
                                nsync = 1;
                                break
                            end
                        else
                            %
                            % - first=accel w/3||5 bytes
                            % - then A but appears we lost A sample, so sync
                            %      is ok so we process first accel sample
                            nsync = 1;
                            break;
                        end
                    end
                end

            elseif isequal(b_start(b_start_indx),'G') 
                %% gyro (first)
                %%==========================
                    % size will be 2 bytes for GH,GL or 4 if there are two break
                    % characters in there also
                if isequal(size(a,1),2) || isequal(size(a,1),4)
                    if isequal(b(1),'G')
                        %% gyro (next)
                        %%==================
                        if size(b,1) >= 4
                            if isequal(b(4),'G') || isequal(b(4),'A')
                                %
                                % - first sample = gyro w/ a=2||4 bytes
                                % - gyro next and the next sample after
                                %     that appears to be a gyro or accel
                                nsync = 1;
                                break
                            end
                        else
                            %
                            % - first=gyro w/2||4 bytes
                            % - then G but appears we lost G sample, so sync
                            %      is ok so we process first gyro sample
                            nsync = 1;
                            break;
                        end
                    elseif isequal(b(1),'A')
                        %% accel (next)
                        %====================
                        if size(b,1) >= 5
                            if isequal(b(5),'G') || isequal(b(5),'A')
                                %
                                % - first sample = gyro w/ a=2||4 bytes
                                % - accel next and the next sample after
                                %     that appears to be a gyro or accel
                                nsync = 1;
                                break
                            end
                        else
                            %
                            % - first=gyro w/2||4 bytes
                            % - then A but appears we lost A sample, so sync
                            %      is ok so we process gyro sample
                            nsync = 1;
                            break;
                        end
                    end
                end
                
            else
                if isequal(b(1),'A') || isequal(b(1),'G')
                    %disp(sprintf('SYNC: invalid but next should work...a=%d b=%d',a(1),b(1)));
                else
                    s = sprintf('SYNC: invalid num of bytes: buf(%d)=%d buf(2)=%d Asize=%d, a=%d b=%d',...
                        b_start_indx,b_start(b_start_indx),b_start(2),size(a,1),a(1),b(1));
                    errordlg(s);
                    a'
                end
            end

            b_start = b;
            [a,b] = strtok(b,'AG');
        end
        if isequal(nsync,0)
            errordlg('FATAL: could not sync');
            buf'
            err = 1;
            return;
        end
        resync = 0;
        %disp(sprintf('SYNC: success, start=%d, next=%d',b_start(1),b(1)));

        % now just use 'b' as we think we are in sync
        b = b_start;
        
        % NOTE: sometimes we get thru sync and have 65
        % from a previous accel sample, then 71 and (2) gyro bytes, then
        % back to accel. But the code would interpret this to be an accel
        % sample with 71 and the (2) bytes. it looks correct to the sync
        % code but isn't. so if we see an 'A' and 'G' after sync, assume
        % it's really a gyro sample and strip off the first byte.
        %
        % Looks like it can be any way here, basically if first byte in
        % buffer is an 'A' or 'G' tag followed by another 'A' or 'G' tag
        % and there is another valid tag at the correct spacing for the 2nd
        % tag, we disregard the first tag and go with the second.
        %
        if (isequal(b(1),'A') && isequal(b(2),'G') && (isequal(b(5),10) || isequal(b(5),'G') || isequal(b(5),'A'))) || ...
                (isequal(b(1),'A') && isequal(b(2),'A') && (isequal(b(6),10) || isequal(b(6),'G') || isequal(b(6),'A'))) || ...
                (isequal(b(1),'G') && isequal(b(2),'A') && (isequal(b(6),10) || isequal(b(6),'G') || isequal(b(6),'A'))) || ...
                (isequal(b(1),'G') && isequal(b(2),'G') && (isequal(b(5),10) || isequal(b(5),'G') || isequal(b(5),'A')))
            b = b(2:end);
        end
        
        ac = 0;
        while true
            if isequal(b(1),'A')
                % 3 bytes is accelerometer
                %disp(sprintf('ACCEL:%d %d %d %d\n',b(1),b(2),b(3),b(4)));
                Af = arrayfun(@twos8,b(2:4));
                %disp(sprintf('A:%d %d %d',Af(1),Af(2),Af(3)));
                A = [A;Af'];
                % we pad the gyro buffer every 2nd accel sample since the
                % gyro is sampled 2/3 as fast as accel. this hopes to keep
                % the returned array in "time sync"
                ac = ac + 1;
                if isequal(ac,3)
                    ac = 0;
                    if ~isempty(G)
                        G = [G;G(end)];
                    end
                end
                if size(b,1) >= 5
                    b = b(5:end);
                else
                    break;
                end
            elseif isequal(b(1),'G')
                % 2 bytes is gyrometer
                %disp(sprintf('GYRO:%d %d %d\n',b(1),b(2),b(3)));
                Gf = twos16(bitor(bitshift(b(2),8),b(3)));
                %disp(sprintf('G:%d',Gf));
                G = [G;Gf'];
                if size(b,1) >= 4
                    b = b(4:end);
                else
                    break;
                end
            elseif isequal(b(1),10) 
                % '10' always has a '12' after it
                %disp('BREAK-10');
                if size(b,1) >= 3
                    b = b(3:end);
                else
                    break;
                end
            elseif isequal(b(1),12)
                %disp('BREAK-12');
                if size(b,1) >= 2
                    b = b(2:end);
                else
                    break;
                end
            else
                s = sprintf('RESYNC! b(1)=%d b(2)=%d',b(1),b(2));
                errordlg(s);
                buf = b;
                resync = 1;
                break;
            end
            if isempty(b) || (isequal(b(1),'A') && size(b,1)<4) || (isequal(b(1),'G') && size(b,1)<3)
                break;
            end
        end
    end
end

%% close serial port
function CloseSerial(socket)
    fclose(socket);
     %disp('SERIAL PORT IS CLOSED');
     %delete(socket);
     %clear socket;
end

%% twos complement convert
function [b] = twos_cnvt(xx,bit)
    %xx = str2num(xx);
    if bitget(xx,bit)
        b = -1 * (bitxor(xx,(power(2,bit)-1))+1);
    else
        b = xx;
    end
end

%% twos 8bit
function [b] = twos8(xx)
    %xx = str2num(xf);
    if bitget(xx,8)
        b = -1 * (bitxor(xx,(power(2,8)-1))+1);
    else
        b = xx;
    end
end

%% twos 10bit
function [b] = twos10(xx)
    %xx = str2num(xf);
    if bitget(xx,10)
        b = -1 * (bitxor(xx,(power(2,10)-1))+1);
    else
        b = xx;
    end
end

%% twos 16bit
function [b] = twos16(xx)
    %xx = str2num(xf);
    if bitget(xx,16)
        b = -1 * (bitxor(xx,(power(2,16)-1))+1);
    else
        b = xx;
    end
end

