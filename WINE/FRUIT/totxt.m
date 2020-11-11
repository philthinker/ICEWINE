function [] = totxt(data,digit,decimal,filename )
%totxt Output the data from workspace to a txt file
%   data: the data to be out
%   digit: the total digits
%   decimal: the digits of the decimal
%   filename:   String, the file name without .txt (default:data)

if nargin < 4
    filename = 'data';
end

interval = 32;  % space

if decimal < digit
    fid = fopen(strcat(filename,'.txt'),'w');
    for i = 1:size(data,1)
        for j = 1:size(data,2)
            formatSpec = strcat('%',int2str(digit),'.',int2str(decimal),'f',interval);
            fprintf(fid,formatSpec,data(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);
end

end

