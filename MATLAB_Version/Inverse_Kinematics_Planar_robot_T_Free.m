close all
clear all
clc

%install_package

while 1

    global theta
    global theta_traj
    global l
    global cart_traj
    global Time
    global Traj_time
    global MyIndex

    % Initial parameters
    N=4;
    l=ones(N,1);
    Print =0;



    % string options
    options = {"Method"};

    % Number of options
    num_options = length(options);
    fprintf('Default option is: method="continuous".\n');
    fprintf('Press Enter button for the default value.\n');
    % Loop through each option
    for i = 1:num_options
        % Display the current option

        fprintf('Current option: %s\n', options{i});

        % Choose an action based on the current option
        switch options{i}
            case "Method"
                fprintf('You have two choices, preses 1 for "continuous" or 2 for "discrete".\n');

                choice = input('Enter your choice: ', 's');
                if isempty(choice)
                    method = 'continuous';
                else
                    choice_num = str2double(choice);
                    if choice_num==1,
                        method = 'continuous';
                    else
                        method = 'discrete';
                    end
                end

        end

    end

    VeloMax=0.5; % joint velcoity upper limit

    theta0 =[0.3763 -0.2270 -1.1489 2.0243]'; %Initial configuration
    theta=theta0;

    dtheta=zeros(size(theta));

    Pe=fd(theta,l);
    Plot_robot(Pe,false,2);

    Jp=JacobianMatrix(theta,Pe);


    Pd0= Pe(end).p(1:2); % Initial end-effector position
    Pd1= Pd0+ [1.5 0.6]'; %Control point
    Pd2= Pd0- [1 2]'; % goal position

    %generate Bezier curve
    ds=1e-3; % sampling time to generate the curve
    scale=0:ds:1;
    traj(:,1)=(1-scale).^2.*Pd0(1) + 2*(1-scale).*scale.*Pd1(1) + scale.^2.*Pd2(1);
    traj(:,2)=(1-scale).^2.*Pd0(2) + 2*(1-scale).*scale.*Pd1(2) + scale.^2.*Pd2(2);
    traj=traj';

    dt=1e-3; % sampling time

    Time=0;

    Traj_time=[];
    cart_traj=Pe(end).p;


    for k=2:length(traj),

        InverseKinematicsControl_T_Free(traj(:,k),dt,k, VeloMax, method, 100,false);

    end;

    P=fd(theta,l);
    Plot_robot(P,true,2);

    plot(Pe(end).p(1),Pe(end).p(2), 'ok','markersize', 14);

    plot(Pd1(1),Pd1(2), 'ok','markersize', 14);
    plot(Pd2(1),Pd2(2), 'ok','markersize', 14);
    plot(cart_traj(1,:),cart_traj(2,:), 'c','LineWidth',2);


    Time_axis=zeros(length(Traj_time),1);
    theta_traj_ext=theta0;

    for j=1:length(Traj_time),

        Time_axis(j+1)=Traj_time(j).time;
        theta_traj_ext(:,j+1)=Traj_time(j).traj;

    end;

    Time_scale =Time_axis(1):dt:Time_axis(end);
    theta_traj=[];


    dtheta_traj_ext=[];

    for i=1:length(theta_traj_ext)-1,
        dtheta_traj_ext(i,:)=(theta_traj_ext(:,i+1)-theta_traj_ext(:,i))./(Time_axis(i+1)-Time_axis(i));
    end;


    figure
    plot(Time_axis(1:end-1),dtheta_traj_ext');
    ylabel('$Rad s^{-1}$');
    xlabel('Time(s)');
    title('joint velocities')
    grid
    hold on
    plot(Time_axis, VeloMax*ones(size(Time_axis)),'r--');
    plot(Time_axis, -VeloMax*ones(size(Time_axis)),'r--');
    axis([0 Time_axis(end) -1.1*VeloMax 1.1*VeloMax]);


    cart_traj=cart_traj(1:2,:);

    figure
    ylabel('m');
    xlabel('Time(s)');
    title('%Cartesian Trajectory')
    grid
    hold on
    plot(cart_traj(1:2,:)','r');
    plot(traj(1:2,:)','g');

    figure,plot(Time_axis, cart_traj(1:2,:)'-traj(1:2,:)');
    ylabel('m');
    xlabel('Time(s)');
    grid
    legend('$Error in x$','$Error in y$','Location','northeast');


    fprintf('Press 1 to continue or any key to exit.\n');

    choice = input('Enter your choice: ', 's');
    choice_num = str2double(choice);

    if isnan(choice_num)
        break
    elseif (choice_num==1)
        close all
        clear all
        clc
    else
        break
    end

end
