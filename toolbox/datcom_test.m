% --- MATLAB Script to Run DATCOM and Check Output ---

clear; clc; close all;

%% 1. Setup File and Executable Names
datcom_executable = 'datcom.exe';
input_filename = 'for005.dat';
output_filename = 'astdatcom.out'; % Or 'for006.out' depending on your DATCOM version

% Check if the DATCOM executable exists in the current folder
if exist(datcom_executable, 'file') ~= 2
    error('"%s" not found in the current directory. Please add it and try again.', datcom_executable);
end

%% 2. Generate the Input File ('for005.dat')
% This section creates a simple DATCOM input file for a basic wing.
disp('Generating DATCOM input file...');
fileID = fopen(input_filename, 'w');

fprintf(fileID, 'CASEID MATLAB Test Case\n');
fprintf(fileID, '$FLTCON NMACH=1.0, MACH(1)=0.5, NALT=1.0, ALT(1)=10000. $\n');
fprintf(fileID, '$FLTCON NALPHA=3, ALSCHD(1)=0.0, 2.0, 4.0 $\n');
fprintf(fileID, '$OPTINS SREF=400.0, CBARR=10.0, BLREF=40.0 $\n');
fprintf(fileID, '$WGPLNF CHRDR=12.0, CHRDTP=8.0, SSPN=20.0, SAVSI=25.0, TYPE=1.0 $\n');
fprintf(fileID, 'NACA-W-4-0012\n');
fprintf(fileID, 'DAMP\n');
fprintf(fileID, 'NEXT CASE\n');

fclose(fileID);
disp('Input file created successfully.');

%% 3. Run the DATCOM Executable
% The 'system' command executes the program. It will pause MATLAB until
% DATCOM is finished. A status of 0 means it ran without OS-level errors.
disp('Running DATCOM... (MATLAB will pause)');

%status = system('./datcom.exe < for005.dat');
%status = system('./datcom.exe');
status = system('echo for005.dat | ./datcom.exe');



disp('DATCOM execution finished.');

%% 4. Check for Output and Parse (Basic Example)
if status == 0
    fprintf('DATCOM process completed with status 0 (Success).\n');
    
    % Check if the output file was created
    if exist(output_filename, 'file') == 2
        fprintf('Output file "%s" found!\n\n', output_filename);
        
        % --- Placeholder for Parsing Logic ---
        % In a real script, you would add code here to open and read
        % 'astdatcom.out' to extract the C_L, C_D, C_m tables.
        disp('--- Basic Parsing Example ---');
        disp('Reading first 15 lines of the output file:');
        fileID_out = fopen(output_filename, 'r');
        for i = 1:15
            line = fgetl(fileID_out);
            if ischar(line) % Check if end of file is reached
                disp(line)
            end
        end
        fclose(fileID_out);
        % ------------------------------------
        
    else
        fprintf('DATCOM ran, but the output file "%s" was not found.\n', output_filename);
    end
else
    fprintf('Error running DATCOM. System status: %d\n', status);
end