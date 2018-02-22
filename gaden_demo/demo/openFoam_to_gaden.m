% -------------------------------------------------------------------------
%                           OpenFoam to GADEN
%--------------------------------------------------------------------------
% This script is part of the GADEN simulator:
% J. Monroy, V. Hernandez-Bennetts, H. Fan, A. Lilienthal, J. Gonzalez-Jimenez,
% GADEN: A 3D Gas Dispersion Simulator for Mobile Robot Olfaction in Realistic Environments
% MDPI Sensors, vol. 17, no. 7: 1479, pp. 1--16, 2017, 
%
% The scripts has 3 main objectives:
%
% 1.To estimate the wind vector (u,v,w) at each cell of a perfect 3D grid.
%   We asume that the input data to this script is a list of 3D points (x,y,z) with 
%   wind flow vectors (u,v,w). This data corresponds to the wind flow at the cell 
%   centers as obtained with CFD, for example with OpenFOAM. This data-format is 
%   easily obtained with ParaView, using the "cell_centers" filter.
%   Since OpenFOAM can alter the shape and size of the cells, in this script
%   we reformat the data to fall in a perfect 3D cube-based grid. This task will
%   generate three files for every wind flow snapshot coming from ParaView: *.csv_U,
%   *.csv_V, *.csv_W, which are the current inputs to the “gaden_filament_simulator” ROS_pkg.
%
% 2.To generate an image representing the 2D view of the environment, useful for
%   navigation purposes where a “map” of the environment is required (e.g. when using the
%   map_server ROS pkg).
%
% 3.To export a 3D occupancy matrix (​OccupancyGrid3D.csv​) where each cell can take the
%   values (0=free, 1=occupied or 2=outlet). This 3D occupancy matrix is necessary for a
%   proper simulation of the gas dispersion on later stages.
%---------------------------------------------------------------------------------------
% NOTE-> This script is tuned for the "gaden_demo" environment provided with GADEN
%---------------------------------------------------------------------------------------

close all;
clear;
clc;


% CONFIGURATION PARAMS
%==========================================================================
% Global volume of the environment (Bounding Box)
% refer to original CAD model used for CFD simulation
% This model only accounts for the inner volume (no walls neither obstacles)
min_range = [-5 -5.5 0];    %m (x,y,z)
max_range = [5 5.5 3];      %m (x,y,z)
plot_data = true;
z_layer_to_plot = 15;       %if plot_data=true, the cell_z to plot 2D slices

% Generate a 3Dgrid with cube cells
cell_resolution = 0.1;      %(m/cell) cell size of desired cube_cells
folder_path = './';         %Path to the folder containing the CFD wind_flow simulations (for this demo is the current folder)

%==========================================================================

% Environment dimensions
environment_dimensions = [max_range(1)-min_range(1) max_range(2)-min_range(2) max_range(3)-min_range(3)]; %(m) 
center_of_environment = [min_range(1)+environment_dimensions(1)/2 min_range(2)+environment_dimensions(2)/2 min_range(3)+environment_dimensions(3)/2];
% Environment in cells
num_cells_x = ceil(environment_dimensions(1)/cell_resolution);
num_cells_y = ceil(environment_dimensions(2)/cell_resolution);
num_cells_z = ceil(environment_dimensions(3)/cell_resolution);


%-----------------------------------------
% 1. Load data from cell_centers CSV file
%-----------------------------------------
% OpenFoam export a complex format, with wind values (u,v,w) for every cell in the env.
% Since the shape of these cells is not constant, we apply a PraView filter to obtain 
% the cell centers, and then export the data to a CSV file (input for this script)
% FORMAT: "U [m/s]:0","U [m/s]:1","U [m/s]:2","Points:0","Points:1","Points:2"
% FORMAT: U, V, W, x, y, z
filename = strcat(folder_path,'wind_at_cell_center_points.csv');
CSV = dlmread(filename,',',1,0);

% Plot original data in 3D - scatter3(x,y,z,size,[color])
% Original data is a set of points with (u,v,w) data
%---------------------------------------------------------
%OPTION: plot only one layer in Zdim
%mask1 = CSV(:,6) > 0.5;
%mask2 = CSV(:,6) < 0.6;
%CSV = CSV(mask1&mask2,:);
if plot_data
    figure();
    scatter3(CSV(:,4), CSV(:,5), CSV(:,6),4,CSV(:,1:3));
    xlabel('X(m)');
    ylabel('Y(m)');
    zlabel('Z(m)');
    set(gca,'xtick',min_range(1):cell_resolution:max_range(1));
    set(gca,'ytick',min_range(2):cell_resolution:max_range(2));
    set(gca,'ztick',min_range(3):cell_resolution:max_range(3));
end;



%----------------------------------------------------------------------------
% 2. Generate 3D Matrices (with regular cell size), for U,V,W and Occupancy3D
%-----------------------------------------------------------------------------

% Wind speed matrix (U,V,W)
U = zeros(num_cells_x,num_cells_y,num_cells_z);
V = zeros(num_cells_x,num_cells_y,num_cells_z);
W = zeros(num_cells_x,num_cells_y,num_cells_z);

% Cell counters (account for several wind speed measurements falling into the same U,V,W cell)
U_cnt = zeros(num_cells_x,num_cells_y,num_cells_z);
V_cnt = zeros(num_cells_x,num_cells_y,num_cells_z);
W_cnt = zeros(num_cells_x,num_cells_y,num_cells_z);

% Enviroment occupancy map
Env = ones(num_cells_x,num_cells_y,num_cells_z);    %Initialize it as full occupied


% For every point in the cell_centers.csv data, add it to its corresponding cell grid
for idx=1:size(CSV,1)
    point = CSV(idx,4:6);
    % cell (x,y,z) corresponding to given point?
    x_idx = ceil( (point(1)-min_range(1))/cell_resolution );
    y_idx = ceil( (point(2)-min_range(2))/cell_resolution );
    z_idx = ceil( (point(3)-min_range(3))/cell_resolution );
    
    % Update wind matrices
    U(x_idx,y_idx,z_idx) = CSV(idx,1);
    V(x_idx,y_idx,z_idx) = CSV(idx,2);
    W(x_idx,y_idx,z_idx) = CSV(idx,3);
    
    % Count how many measurents have fallen into a cell
    U_cnt(x_idx,y_idx,z_idx) = U_cnt(x_idx,y_idx,z_idx)+1;
    V_cnt(x_idx,y_idx,z_idx) = V_cnt(x_idx,y_idx,z_idx)+1;
    W_cnt(x_idx,y_idx,z_idx) = W_cnt(x_idx,y_idx,z_idx)+1;
    
    % Set occupancy as free (since there is wind info)
    Env(x_idx,y_idx,z_idx) = 0;
end;


% Average windspeed in U,V,W cells according to theirmeasurement count
U = U./U_cnt;
V = V./U_cnt;
W = W./U_cnt;


% Plot interpolated data in 3D
if plot_data
    figure();
    wind_intensity = sqrt(U(:,:,z_layer_to_plot).^2 + V(:,:,z_layer_to_plot).^2 + W(:,:,z_layer_to_plot).^2 );
    Xvals = min_range(1)+cell_resolution/2:cell_resolution:max_range(1);
    Yvals = min_range(2)+cell_resolution/2:cell_resolution:max_range(2);
    [X,Y] = meshgrid(Xvals,Yvals);
    surf(X,Y,wind_intensity');
    xlabel('X(m)');
    ylabel('Y(m)');
    zlabel('Z(m)');
end;


%-----------------------------------------
% 3. DECLARE OUTLETS (window/door)
%-----------------------------------------
% Outlets are regions of the CAD model where filaments leave the
% environment. As in the case of CFD, outlets are occupied cells but with an special
% treatement in the gaden_filament_disperion pkg (disables the filaments)
% Cell occupancy states: 0=free cell, 1=occupied cell, 2=outlet
if plot_data
    figure();
    subplot(1,2,1);
    title('Occupancy map');
    hold on;
    M = Env(:,:,z_layer_to_plot);
    % Find the location of the ones (occupied cells)
    [row, col] = find(M>=1);
    plot(row, col, 'ob');
    xlabel('Xcell');
    ylabel('Ycell');
end;

% ToDo: is it possible to get this information directly from CFD results?
%Outlet_1: left door ->  20<Xcell<30, 108<Ycell and Zcell<20
Env(20:30,108:end,1:20) = 2;
%Outlet_2: right door -> 70<Xcell<80, Ycell>108 and Zcell<20
Env(70:80,108:end,1:20) = 2;
    

if plot_data    
    subplot(1,2,2);
    title('New Occupancy map');
    hold on;
    M = Env(:,:,z_layer_to_plot);
    % Find the location of the ones (occupied cells)
    [row, col] = find(M==1);
    plot(row, col, 'ob');
    % Find the location of the twos (outlet cells)
    [row, col] = find(M==2);
    plot(row, col, '*r');
    xlabel('Xcell');
    ylabel('Ycell');
end;



%------------------------------
% 4. Save data (GADEN format)
%------------------------------
% Each component of the wind (u,v,w) is saved to a different file.
% Each file contains the wind flow components for all 3D cells of the environment
% To do so, we write 2D layers of the map, increasing Zdim

file_U = strcat(filename,'_U');
if exist(file_U, 'file')==2
  delete(file_U);
end
file_V = strcat(filename,'_V');
if exist(file_V, 'file')==2
  delete(file_V);
end
file_W = strcat(filename,'_W');
if exist(file_W, 'file')==2
  delete(file_W);
end
file_Env = strcat(folder_path,'OccupancyGrid3D.csv');
if exist(file_Env, 'file')==2
  delete(file_Env);
end

% Add header to the 3DOccupancy file.
fileID = fopen(file_Env,'w');
fprintf(fileID,'#env_min(m) %.4f %.4f %.4f\n', min_range(1), min_range(2), min_range(3));
fprintf(fileID,'#env_max(m) %.4f %.4f %.4f\n', max_range(1), max_range(2), max_range(3));
fprintf(fileID,'#num_cells %i %i %i\n', num_cells_x, num_cells_y, num_cells_z);
fprintf(fileID,'#cell_size(m) %.4f\n', cell_resolution);
fclose(fileID);

for j=1:size(U,3)
    % write Zlayer=j from U matrix to file
    dlmwrite(file_U, U(:,:,j) ,'-append','delimiter',' ');    
    fileID = fopen(file_U,'a'); % Delimiter for next Zlayer
    fprintf(fileID,';\n');
    fclose(fileID);

    % write V matrix to file
    dlmwrite(file_V, V(:,:,j), '-append','delimiter',' ');
    fileID = fopen(file_V,'a');
    fprintf(fileID,';\n');
    fclose(fileID);

    % write W matrix to file
    dlmwrite(file_W, W(:,:,j), '-append','delimiter',' ');
    fileID = fopen(file_W,'a');
    fprintf(fileID,';\n');
    fclose(fileID);
    
     % write Env matrix to file
    dlmwrite(file_Env, Env(:,:,j), '-append','delimiter',' ');
    fileID = fopen(file_Env,'a');
    fprintf(fileID,';\n');
    fclose(fileID);
end


%-----------------------------------------
% 5. Save a 2D bitmap of the environment
%-----------------------------------------
% This bitmap is useful for robotic navigation (e.g. ROS map_server)

z_layer_for_occ = 5
env = Env(:,:,z_layer_for_occ)>=1;  %set value 1 for obstacles and outlets

% Traspose Matrix
env = env';

%Invert 0/1 free/occupied
env = 1-env;

%flip image
env = flip(env ,1);           %# vertical flip

% To image
I = mat2gray(env);
figure();
subplot(1,2,1);
imshow(I);

%Change Resolution to get a bigger image
final_res = 0.01;   %m/px
subplot(1,2,2);
I2 = imresize(I,cell_resolution/final_res,'nearest');
imshow(I2);
% save to file
imwrite(I2,'OccupancyMap2D.pgm');
