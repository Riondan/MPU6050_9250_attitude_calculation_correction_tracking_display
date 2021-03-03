function devices = IdentifySerialComs()
    devices = [];
    Skey = 'HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM';
    [~, list] = dos(['REG QUERY ' Skey]);
    if ischar(list) && strcmp('ERROR',list(1:5)) 
        disp('Error: EnumSerialComs - No SERIALCOMM registry entry')
        return;
    end
    list = strread(list,'%s','delimiter',' '); %requires strread()
    coms = 0;
    for i = 1:numel(list)  %numel
        if strcmp(list{i}(1:3),'COM')
            if ~iscell(coms)
                coms = list(i);
            else
                coms{end+1} = list{i}; %Loop size is always small
            end
        end
    end
    out = 0;
    outK = 0;
    for j=1:2
        switch j
            case 1
                key = 'HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB\';
            case 2
                key = 'HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\FTDIBUS\';
        end
        [~, vals] = dos(['REG QUERY ' key ' /s /f "FriendlyName" /t "REG_SZ"']);
        if ischar(vals) && strcmp('ERROR',vals(1:5))
            disp('Error: EnumSerialComs - No Enumerated USB registry entry')
            return;
        end
        vals = textscan(vals,'%s','delimiter','\t');
        vals = cat(1,vals{:});
        for i = 1:numel(vals)
            if strcmp(vals{i}(1:min(12,end)),'FriendlyName')
                if ~iscell(out)
                    out = vals(i);
                else
                    out{end+1} = vals{i}; %Loop size is always small
                end
                if ~iscell(outK)
                    outK = vals(i-1);
                else
                    outK{end+1} = vals{i-1}; %Loop size is always small
                end
            end
        end
    end

    i_dev=1;Sservices=[];
    for i = 1:numel(coms)
        match = strfind(out,[coms{i},')']);
        ind = 0;
        for j = 1:numel(match)
            if ~isempty(match{j})
                ind = j;
                [~, sers] = dos(['REG QUERY "' outK{ind} '" /f "Service" /t "REG_SZ"']);
                sers = textscan(sers,'%s','delimiter','\t');
                sers = cat(1,sers{:});
                if (numel(sers)>1)
                    sers=strread(sers{2},'%s','delimiter',' ');
                    Sservices{i_dev} = sers{3};
                    i_dev=i_dev+1;
                end
            end
        end
    end
    Sservices=unique(Sservices);

    i_dev=1;
    for ss=1:numel(Sservices)
        key = ['HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\services\' Sservices{ss} '\Enum'];
        [~, vals] = dos(['REG QUERY ' key ' /f "Count"']);
        if ischar(vals) && strcmp('ERROR',vals(1:5))
    %         disp('Error: EnumSerialComs - No Enumerated services USB registry entry')
    %         return
        end
        vals = textscan(vals,'%s','delimiter','\t');
        vals = cat(1,vals{:});

        if (numel(vals)>1)
            vals=strread(vals{2},'%s','delimiter',' ');
            Count=hex2dec(vals{3}(3:end));
            if Count>0
                [~, vals] = dos(['REG QUERY ' key]);
                vals = textscan(vals,'%s','delimiter','\t');
                vals = cat(1,vals{:});
                out=0;
                j=0;
                for i = 1:numel(vals)
                    Enums=strread(vals{i},'%s','delimiter',' ');
                    try nums=hex2dec(Enums{1});
                    catch
                        nums=-1;
                    end
                    if(nums==j)
                        out=['HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\' Enums{3}];
                        [~, listC] = dos(['REG QUERY "' out '" /s /f "PortName" /t "REG_SZ"']);
                        listC = textscan(listC,'%s','delimiter','\t');
                        listC = cat(1,listC{:});
                        if (numel(listC)>1)
                            listC=strread(listC{2},'%s','delimiter',' ');
                            for i = 1:numel(coms)
                                if strcmp(listC{3},coms{i})
                                    [~, NameF] = dos(['REG QUERY "' out '" /s /f "FriendlyName" /t "REG_SZ"']);
                                    NameF = textscan(NameF,'%s','delimiter','\t');
                                    NameF = cat(1,NameF{:});
                                    com = str2double(coms{i}(4:end));
                                    if com > 9
                                        length = 8;
                                    else
                                        length = 7;
                                    end
                                    devices{i_dev,1} = NameF{2}(27:end-length); 
                                    devices{i_dev,2} = com;
                                    i_dev=i_dev+1;
                                end
                            end
                        end
                        j=j+1;
                    end
                end
            end
        end
    end
end