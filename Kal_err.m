function [ MAPE_Kalman, MAPE ] = Kal_err(x,subject,hide_figures,calib_days)

%% =====================================================================
% Kal_err returns error between predicted performance curve on given
% 2-timeconstant-Fitness-Fatigue Model (IJCSS-Vol7 Pfeiffer pg.22-23 )
% with Kalman filter parameters and
% recorded observations of performance level of one testSubject.
%
% Choose function output among - MAPE, "MAPE_Kalman", sq_sum_kal,
%                                   sq_sum_det, sq_sum_interp
%
% Function arguments are ordered as
% (x=[tau1 tau2 k1 k2 sigma_statenoise fitness_0 fatigue_0])
%
% tau1 = Time constant for Fitness decay
% tau2 = Time constant for Fatigue decay
% k1 = Magnitude factor for fitness
% k2 = Magnitude factor for fatigue
% sigma_statenoise = Standard deviation of noise in fitness and fatigue
% values
% fitness_0 = Initial value of fitness state
% fatigue_0 = Initial value of fatigue state
%
%  subject = Chosen integer from 1 to 5 (Proband# in original data)
%  hide_figures-optional.if provided, suppresses graphics (for optim calls)
%  calib_days is optional parameter to limit no. of days for sim run
%
% For example   Kal_err([10 5 .005 .01 30 500 200],1) gives MAPEs for
% Subject 1.
%
%
%
%
%==========================================================================
%==========================================================================
do_debug = 0;

% TrainingData_read.mat: file containing all training inputs and
% performance measurements.
% Created from by script "import_trainingdata.m" which imports data
% from .txt files containing raw data
trainingData = load('TrainingData_read.mat');

%Select Test Subject and endpoint for fitting data - (0 for complete range)
testSubject = subject;
if ~exist('calib_days','var')
    end_fit=inf;
    calib_days = inf;
else
    end_fit = calib_days;
end
%==========Truncate experimental data for calibration============
if end_fit<inf
    In=trainingData.Athlete_In{testSubject}(1:end_fit);
    Out=trainingData.Athlete_Out{testSubject}(1:end_fit);
    T_end =end_fit;
else
    In=trainingData.Athlete_In{testSubject};
    Out=trainingData.Athlete_Out{testSubject};
    T_end = length(In); %---Sim Time-------
end
%===============================================================

T = 1;  % Simulation time step = 1 day


%=======================Model parameters========================
tau1 =  x(1);
tau2 = x(2);
k1 = x(3);
k2= x(4);
%===============================================================


%=========STATE SPACE MODEL====== Y= Base_performance + C * x , x=A*x + B*u
A = [exp(-1/tau1)  0 ; ...
    0        exp(-1/tau2)];

b1 = 1*exp(-1/tau1);
b2 = 1*exp(-1/tau2);

B = [b1 ; b2 ];
C = [k1 , -k2];
%====================Kalman Filter Parameters==============================

sigma_Sx = x(5);        % Stdev of State noise
sigma_Px = .0126;       % Stdev of Observed Performance
% .0126 is 1% of average performance value

% Q = sigma_Sx^2 * [b1^2, b1*b2;...  %State noise covariance matrix
%     b2*b1  b2^2];               % E(v_k . v_k') = B*B'sigma_sx^2
Q= sigma_Sx^2 * [1 0;0 1];  %Independent Fitness & Fatigue


R = sigma_Px^2;                    % E(n_k . n_k')

%==========================================================================

x0hat = [x(6) , x(7)]';         % Initial conditions for filter
x_0 =   [x(6) , x(7)]';         % Init conditions for deterministic model
M_0 =   Q;                      % Init covariance M is Q


%============================Simulink Inputs===============================
t = [1:T:T_end]';           %Time vector
u_k = In;                   %Training Input to system
base_perf = Out(1);         %Model performance o/p is change from this value
yout_k= Out - base_perf ;   %Simulation begins from initally known
%performance i.e. 1st obs

%================RUN AND RETRIEVE SIMULATION RESULTS=======================
options = simset('SrcWorkspace','current');
SimOut = sim('FF_Kalman_Without_Init',[],options);


new = SimOut.get('yout');       %Simulation results stored in struct
Kalman_err = reshape(new.signals(2).values,1,size(new.signals(2).values,3));
sq_Kal_err= Kalman_err.^2;      %Sum of squared error from KF output

det_model_err = new.signals(3).values ;
sq_det_err = det_model_err.^2;  %Squared error sum from output w/o feedback

%=============Get states evolution=============
fit_kal = reshape(new.signals(4).values(1,1,:),...
    size(new.signals(4).values(1,1,:),3),1);

fat_kal = reshape(new.signals(4).values(2,1,:),...
    size(new.signals(4).values(1,1,:),3),1);

fit_det = reshape(new.signals(4).values(3,1,:),...
    size(new.signals(4).values(1,1,:),3),1);

fat_det = reshape(new.signals(4).values(4,1,:),...
    size(new.signals(4).values(1,1,:),3),1);
%================================================

sq_sum_kal = sum(sq_Kal_err); %With and w/o feedback errors
sq_sum_det = sum(sq_det_err);


%========== Separate the true observation instants(i.e. not zero)======
% Use available performance observations to calculate MAPE.
% All yout values having the value -base_perf are those at which a
% performance measurement is not available.

yout_non_negative=[];
yout_non_negative_orig_scale=[];

for i=1:1:T_end
    yout_non_negative_orig_scale(i) = yout_k(i) + base_perf; %For MAPE calc
    
    if yout_k(i) == -base_perf;              %If the original value is zero
        yout_non_negative_orig_scale(i) =1; %avoid div by zero in MAPE calc
    end
end

%========= Calculation of MAPE from Deterministic model==================
frac_err=abs(det_model_err./yout_non_negative_orig_scale');%Fractional error
MAPE= sum(frac_err)/length(find(frac_err>0)); %Div by number of nonzero obs

%========== Similarly KF MAPE ==========================================
frac_err_kalman = abs(Kalman_err./yout_non_negative_orig_scale);
MAPE_Kalman = sum(frac_err_kalman)   /   length(find(frac_err>0));



if ~exist('hide_figures')   %Show if hide_figures is not supplied
    %================================Plots===============================
    subplot(211);
    hold off;
    plot(t,new.signals(1).values);         % This is y_out - deterministic
    hold on;grid on;set(gca,'Color',[0.3 0.3 0.3]);%ylim([-.15 .4]);
    set(gca,'XTick',t(frac_err>0));
    filtered_perf=reshape(new.signals(5).values(1,1,:),...
        size(new.signals(5).values(1,1,:),3),1);
    plot(t,filtered_perf,'r');           %This KF output
    plot(t,yout_k,'-yo');                %Measured outputs from experiments
    plot(t,In/100 - 0.8,'k+');           %Training input scaled,shifted
    legend('Deterministic Model','KF Output','Observations','Training Input');
    
    
    subplot(212);
    plot(t,fat_det,'r-.');                %Fatigue no feedback
    grid on;hold on;set(gca,'Color',[0.3 0.3 0.3]);
    set(gca,'XTick',t(frac_err>0));
    plot(t,fat_kal,'m');                  %Fatigue KF
    plot(t,fit_det,'g-.');
    plot(t,fit_kal,'y');
    % ax.set_yticks([0.3,0.55,0.7]);
    legend('Fatigue - Det','Fatigue - KF','Fitness - Det','Fitness - KF');
    
    title(['MAPE: ',num2str(MAPE),' Kalman MAPE: ',num2str(MAPE_Kalman),...
        ' sigma v_k=',num2str(sigma_Sx),...
        '   Subject=',num2str(testSubject),' tau1=',...
        num2str(tau1),' tau2=',num2str(tau2),...
        ' k1=',num2str(k1),' k2=',num2str(k2)]);
    
    if do_debug == 1
        %% ========This part plots evolution of Kalman Gain====================
        kGains = SimOut.get('K_gain');
        kal_gain=[];
        kal_gain(:,1) = kGains.signals.values(1,:,:);
        kal_gain(:,2) = kGains.signals.values(2,:,:);
        figure;
        plot(kal_gain);
        title(strcat('Evolution of Kalman Gain - subj: ',num2str(testSubject)));
        legend('Fitness KGain','Fatigue KGain');
        %%=====================================================================
        %% Cov matrix details
        
        covar_subj=SimOut.get('Mk1');
        M_subj=covar_subj.signals.values;
        M_end.(['sub' num2str(testSubject)])= M_subj(:,:,length(M_subj));
        M_end= M_subj(:,:,length(M_subj));
        inv(R+C*M_end*C');
        C'*inv(R+C*M_end*C');
        gain = M_end*C'*inv(R+C*M_end*C');
        gains_end.(['sub' num2str(testSubject)]) = M_end*C'*inv(R+C*M_end*C');
        eig(M_end);
    end
    
end

end

