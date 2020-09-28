%% This is the main script for Kalman Filter Implementation on the
%% Fitness_Fatigue Model to our observed data. It does the following:
%% 1. Import data data to mat file from provided text files (calls another script)
%% 2. Call optimizer for FF parameters
%% 3. Display results 
%%
%% Authors: Bin Azhar, Kolossa
%% Mail: dorothea.kolossa@rub.de


close all; clear vars;

% ===========Changeable parameters==========================
calib_days= inf;            % No. of days to run calibration, inf: use all
max_time = 120;             % Maximum optimization time [sec] per subject
num_athletes = 1;           % number of athletes
path_to_data = '.\Data\';          % Directory with data files
name_stub = 'Fictitious_Athlete_'; % filename pattern: Fictitious_Athlete_i


%% (1)Import data from txt file to mat
import_trainingdata(num_athletes, path_to_data, name_stub);

%% (2)Start 7-parameter calibration on full range (i.e. from day 0)
%% of data using multi start optimization

% See Kal_err.m for further details
%
% Elements in x meaning : [tau1 tau2 k1 k2 sigma_Sx fitness_0 fatigue_0]
% tau1 = Time constant for Fitness decay
% tau2 = Time constant for Fatigue decay
% k1 = Magnitude factor for fitness
% k2 = Magnitude factor for fatigue
% sigma_statenoise = Std. dev. of noise in fitness and fatigue values
% fitness_0 = Initial value of fitness state
% fatigue_0 = Initial value of fatigue state

hide_figures=1;     %Suppress figures for optimization calls
x_start_optim = [20 10 .01 .01 0.5 0 0];  %Starting point based on
                                        %values from previous studies
                                        %Pfeiffer pg.27
lb = [1 1 .001 .001 0.01 0 0];             %Lower bound
ub = [50 50 .1 .1 5 1000 1000];        %Upper bound
Aineq = [-1 1.5 0 0 0 0 0];             %Constraint tau1 > 1.5 * tau2
bineq = [0]; %#ok<NBRAK>

%Start calibration
for(testSubj=1:1:num_athletes)
    disp(['FF Model Fitting to all measured data for subject ',num2str(testSubj)]);

    f=@(x)Kal_err(x,testSubj,hide_figures,calib_days);  %Objective function
    ms = MultiStart('MaxTime',max_time);        %MS object
    
    problem = createOptimProblem('fmincon','objective',f,'x0',...
        x_start_optim,'lb',lb,'ub',ub,'Aineq',Aineq,'bineq',bineq);
    

    [x_full.(['pars' num2str(testSubj)]),fg,exitflag,output,solutions] =...
        run(ms,problem,25);     %Run optimizer
    
end

%% For calibration using first half (30 days) of observations
calib_days=30;

for testSubj=1:num_athletes
    disp(['FF Model Fitting partial data for subject ',num2str(testSubj)]);

    f=@(x)Kal_err(x,testSubj,hide_figures,calib_days);  %Objective function
    ms = MultiStart('MaxTime',max_time);        %MS object
    
    problem = createOptimProblem('fmincon','objective',f,'x0',...
        x_start_optim,'lb',lb,'ub',ub,'Aineq',Aineq,'bineq',bineq);

    [x_half.(['pars' num2str(testSubj)]),fg,exitflag,output,solutions] =...
        run(ms,problem,25);
    
end

save('ourSettings');

%% Load old settings
settingWithInit= load('ourSettings.mat');

%% (3)DISPLAY RESULTS
%% Parameters fullrange-optimized min Kal_MAPE with initial states

sum_det_MAPE=0;sum_kal_MAPE=0;

for testSubj=1:num_athletes
    figure(); title('Calibrated on all data')
    a=settingWithInit.('x_full');
    a=a.(['pars',num2str(testSubj)]);
    [y1, y2]= Kal_err(a,testSubj); %Kal_err returns [MAPE_Kalman MAPE_det]
    sum_det_MAPE = sum_det_MAPE + y2;
    sum_kal_MAPE = sum_kal_MAPE + y1;
end

disp('Parameters fullrange-optimized min Kal_MAPE with initial states')
avg_det_MAPE = sum_det_MAPE/num_athletes 
avg_kal_MAPE = sum_kal_MAPE/num_athletes 

%% Parameters halfrange-optimized min Kal_MAPE with initial states
sum_det_MAPE=0;sum_kal_MAPE=0;
for testSubj=1:num_athletes
    figure(); title('Calibrated on half the data')
    a=settingWithInit.('x_half');
    a=a.(['pars',num2str(testSubj)]);
    [y1, y2]= Kal_err(a,testSubj); %Kal_err returns [MAPE_Kalman MAPE_det]
    sum_det_MAPE = sum_det_MAPE + y2;
    sum_kal_MAPE = sum_kal_MAPE + y1;
end

disp('Parameters halfrange-optimized min Kal_MAPE with initial states')
avg_det_MAPE = sum_det_MAPE/num_athletes %#ok<*NOPTS>
avg_kal_MAPE = sum_kal_MAPE/num_athletes

