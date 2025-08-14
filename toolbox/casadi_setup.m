% setup_casadi.m
% This script downloads and sets up the correct version of CasADi.

function setup_casadi()
    % --- Configuration ---
    casadi_version = '3.6.5'; % Specify the CasADi version you need
    dest_folder = 'external'; % Destination for downloads
    casadi_folder_name = '';  % Will be determined by OS
    
    % --- Check if already set up ---
    if exist(fullfile(dest_folder, 'casadi'), 'dir')
        fprintf('CasADi seems to be already set up in /%s/.\n', dest_folder);
        addpath(genpath(fullfile(dest_folder, 'casadi')));
        fprintf('Added CasADi to path.\n');
        return;
    end

    % --- OS Detection and URL Definition ---
    if ispc
        fprintf('Operating System: Windows\n');
        zip_file = sprintf('casadi-%s-windows64-matlab2018b.zip', casadi_version);
        casadi_folder_name = sprintf('casadi-%s-windows64-matlab2018b.zip', casadi_version);
    elseif ismac
        fprintf('Operating System: macOS\n');
        zip_file = sprintf('casadi-macos-matlabR2015a-v%s.zip', casadi_version);
        casadi_folder_name = sprintf('casadi-macos-matlabR2015a-v%s', casadi_version);
    elseif isunix
        fprintf('Operating System: Linux\n');
        zip_file = sprintf('casadi-linux-matlabR2014b-v%s.zip', casadi_version);
        casadi_folder_name = sprintf('casadi-linux-matlabR2014b-v%s', casadi_version);
    else
        error('Unsupported operating system.');
    end
    %vhttps://github.com/casadi/casadi/releases/download/3.7.1/casadi-3.7.1-windows64-matlab2018b.zip
    base_url = 'https://github.com/casadi/casadi/releases/download/';
    download_url = [base_url, casadi_version, '/', zip_file];

    % --- Download and Unzip ---
    fprintf('Downloading CasADi from %s...\n', download_url);
    
    % Create destination folder if it doesn't exist
    if ~exist(dest_folder, 'dir')
        mkdir(dest_folder);
    end
    
    % Download and unzip
    zip_path = fullfile(dest_folder, zip_file);
    websave(zip_path, download_url);
    unzip(zip_path, dest_folder);
    
    % Rename the folder to something simple like "casadi"
    movefile(fullfile(dest_folder, casadi_folder_name), fullfile(dest_folder, 'casadi'));
    
    % Clean up the downloaded zip file
    delete(zip_path);

    % --- Add to Path ---
    %addpath(genpath(fullfile(dest_folder, 'casadi')));
    addpath(dest_folder)
    fprintf('CasADi has been downloaded, set up, and added to your MATLAB path.\n');
    
    % Optional: save the path for future sessions
    % savepath; 
    % Note: savepath can sometimes cause issues with user permissions.
    % It's often safer to just have users run this script once.

end