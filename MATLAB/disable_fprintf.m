function disable_fprintf(b_disable) %#codegen

    filenames_m = dir(fullfile(pwd,'**/*.m'));
    for index = 1:size(filenames_m,1)
        
        if ((strcmp(filenames_m(index).name,'disable_fprintf.m') || strcmp(filenames_m(index).name,'generate_code.m'))==false)
        
            filetext = fileread([filenames_m(index).folder,'/',filenames_m(index).name]);
            if (b_disable == true)
                filetext = strrep(filetext,'fprintf','%fprintf');
                filetext = strrep(filetext,'printnum','%printnum');
            else
                filetext = strrep(filetext,'%fprintf','fprintf');
                filetext = strrep(filetext,'%printnum','printnum');
            end
            fid = fopen([filenames_m(index).folder,'/',filenames_m(index).name], 'w');
            fprintf(fid,'%s', filetext);
            fclose(fid);
        end
    end
end