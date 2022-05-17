clc;clear


in_data_version = 'v7_9_combined';
in_dir = ['data' filesep in_data_version filesep];

out_dir = ['data' filesep in_data_version '_preproc' filesep];
mkdir(out_dir)

file_list = dir([in_dir '*.mat']);

for i = 1:length(file_list)
% i = 1
    file_dir = [file_list(i).folder filesep file_list(i).name];
    
    load(file_dir)
    
    data_rear(:,6) = data_rear(:,6) + data_rear(:,9);
    data_rear(:, 22) = 0;
    
    data_front(:, 22) = 1;
    
    data = sortrows([data_rear; data_front], [1, 2]);
    
    save([out_dir file_list(i).name], 'data')
end

%%
