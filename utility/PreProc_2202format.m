% preprocessing

clc;clear

data_dir = ['data' filesep 'rawdata_combined_v7' filesep];

list_us = dir([data_dir '*1234*']);
% list_ds = dir([data_dir '*4567*']);

data_all = cell(12,1);
for i = 1:6
    file_us = [list_us(i).folder filesep list_us(i).name];
    tmp_us = load_data(file_us, 0, 0);
    tmp_us(:,14)= i;

    file_ds = [list_ds(i).folder filesep list_ds(i).name];
    tmp_ds = load_data(file_ds, 1, 0);
    tmp_ds(:,14)= i;
    
    data_all{2*i} = tmp_ds;
    data_all{2*i-1} = tmp_us;
end


%%
vNum = '7';

data = cell2mat(data_all);

save([data_dir 'cam1234_4567_combined_shifted_v' vNum '.mat'], 'data')
