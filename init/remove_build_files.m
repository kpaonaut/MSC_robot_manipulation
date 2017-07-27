%% Wenjie Chen, 2016/06/22, FANUC Corporation
% To remove all the files generated during model compiling and building

function remove_build_files(relPath2Experimentor)

disp('---> Removing all the files generated during model compiling and building...');

if nargin < 1
    relPath2Experimentor = 'experimentor/';
end

rmdir([relPath2Experimentor, 'LRMate200Experimentor_Dual_slrt_rtw'], 's');
rmdir([relPath2Experimentor, 'slprj'], 's');
delete([relPath2Experimentor, 'ecat_config_xml_0*.c']);
delete([relPath2Experimentor, 'LRMate200Experimentor*.dlm']);
delete([relPath2Experimentor, 'LRMate200Experimentor*.xml']);
delete([relPath2Experimentor, 'LRMate200Experimentor*.m']);

fprintf('---> All the files generated during model compiling and building are removed.\n\n');
