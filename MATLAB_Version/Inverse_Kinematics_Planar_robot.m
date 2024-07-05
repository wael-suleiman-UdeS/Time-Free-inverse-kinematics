close all
clear all
clc

%install_package

while 1

    global theta
    global theta_traj
    global l
    global cart_traj

    % Initial parameters
    N=4;
    l=ones(N,1);

    % string options
    options = {"Method", "Total Time"};

    % Number of options
    num_options = length(options);
    fprintf('Default options are: method="conventional" and Total time=4 seconds.\n');
    fprintf('Press Enter button for the default values.\n');
    % Loop through each option
    for i = 1:num_options
        % Display the current option

        fprintf('Current option: %s\n', options{i});

        % Choose an action based on the current option
        switch options{i}
            case "Method"
                fprintf('You have two choices, preses 1 for "conventional" or 2 for "conventional_relaxed".\n');

                choice = input('Enter your choice: ', 's');
                if isempty(choice)
                    method = 'conventional';
                else
                    choice_num = str2double(choice);
                    if choice_num==1
                        method = 'conventional';
                    else
                        method = 'conventional_relaxed';
                    end
                end

            case "Total Time"
                fprintf('Enter the total time for the motion (Tf).\n');
                choice = input('Enter your choice: ', 's');
                choice_num = str2double(choice);
                if isempty(choice)||isnan(choice_num)
                    Tf=4;
                else
                    Tf = choice_num;
                end

        end

    end



    dt=1e-3; % sampling time to generate the curve

    VeloMax=.5; % joint velcoity upper limit

    theta =[0.3763 -0.2270 -1.1489 2.0243]'; %Initial configuration

    dtheta=zeros(size(theta));

    Pe=fd(theta,l);
    Plot_robot(Pe,false,2);

    Jp=JacobianMatrix(theta,Pe);


    Pd0= Pe(end).p(1:2); % Initial end-effector position
    Pd1= Pd0+ [1.5 0.6]'; %Control point
    Pd2= Pd0- [1 2]'; % goal position

    %generate Bezier curve
    time=0:dt/Tf:1;
    traj_scaled(:,1)=(1-time).^2.*Pd0(1) + 2*(1-time).*time.*Pd1(1) + time.^2.*Pd2(1);
    traj_scaled(:,2)=(1-time).^2.*Pd0(2) + 2*(1-time).*time.*Pd1(2) + time.^2.*Pd2(2);
    traj_scaled=traj_scaled';

    time=time*Tf;
    traj=traj_scaled;

    for k=1:length(traj),

        InverseKinematicsControl(traj(:,k),dt,VeloMax,method, 100,false);

    end

    P=fd(theta,l);
    Plot_robot(P,true,2);

    plot(Pe(end).p(1),Pe(end).p(2), 'ok','markersize', 14);

    plot(Pd1(1),Pd1(2), 'ok','markersize', 14);
    plot(Pd2(1),Pd2(2), 'ok','markersize', 14);
    plot(cart_traj(1,:),cart_traj(2,:), 'c','LineWidth',2);

    dtheta_traj=zeros(N,length(theta_traj)-1);

    for i=1:size(theta_traj,1)
        dtheta_traj(i,:)=diff(theta_traj(i,:))./dt;
    end



    figure
    plot(time(1:end-1),dtheta_traj');
    ylabel('$Rad s^{-1}$');
    xlabel('Time(s)');
    title('Joint velocity')
    grid
    hold on
    plot(time, VeloMax*ones(size(time)),'r--');
    plot(time, -VeloMax*ones(size(time)),'r--');
    axis([0 time(end) -1.1*VeloMax 1.1*VeloMax]);

    cart_traj=cart_traj(1:2,:);


    figure
    subplot(2,1,1)
    ylabel('X(m)');
    xlabel('Time(s)');
    title('End effector trajectories')
    grid
    hold on
    plot(time,cart_traj(1,:)','g');
    plot(time,traj(1,:)','r');
    legend('Executed trajectory','Desired trajectory','Location','northeast');

    subplot(2,1,2)
    ylabel('Y(m)');
    xlabel('Time(s)');
    title('End effector trajectories')
    grid
    hold on
    plot(time,cart_traj(2,:)','g');
    plot(time,traj(2,:)','r');
    legend('Executed trajectory','Desired trajectory','Location','northeast');

    figure,plot(time, cart_traj(1:2,:)'-traj(1:2,:)');
    ylabel('m');
    xlabel('Time(s)');
    grid
    legend('Error in x','Error in y','Location','northeast');

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