%% Import/Export Simulink Data from/to Excel

%Export to Excel
%fprintf('Export Started!')
%FileName = fullfile('D:\Dropbox\EE500\CURRENT Model for editing\Excel_Sim_Data\',sprintf('RD_w_AYC_RHT_20mss_%s.xlsx',datestr(now,'HHMMSS_mmddyy')))
%A = [VMC]; 
%xlswrite(FileName,A)
%fprintf('Export Finished!')

%Import to Excel
fprintf('Import Begin!')
VMC = xlsread('C:\Users\Kenny Schmutz\Dropbox\EE500\CURRENT Model for editing\Excel_Sim_Data\RD_w_TCS2_AYC_RHLHRSI_SApm3_45sec_055540_082514.xlsx');
fprintf('\nImport Finished!')