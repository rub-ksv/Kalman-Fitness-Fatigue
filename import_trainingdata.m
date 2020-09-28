function import_trainindata_release(num_subjects,path_to_data,name_stub)
%% Import data from text file.
% Script for importing data from the following text file type:
%
%   \Daten\Proband_*_Datentyp_1.txt
%   Generated on 2015/09/20 13:18:23


for number=1:num_subjects
    
    %% Initialize variables.
    filename = [path_to_data,name_stub, num2str(number),'.txt'];
    
    data = load(filename, '-ascii');
    
    %% Allocate imported array to column variable names
    Athlete_In{number} = data(:, 1);
    Athlete_Out{number} = data(:, 2);
end

save('TrainingData_read.mat');