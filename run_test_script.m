while true
clc
  
% List of files to choose from
files = {"Inverse_Kinematics_Planar_robot.m", "Inverse_Kinematics_Planar_robot_T_Free.m"};
Algorithms={"Conventional Methods", "Proposed Method"};

% Display the options
fprintf('Please choose a method to run:\n');
for i = 1:length(Algorithms)
    fprintf('%d: %s\n', i, Algorithms{i});
end
fprintf('%d: Exit\n', length(files) + 1);

    % Read user input
    choice = input('Enter your choice: ', 's');
    
    % Check if the input is empty
    if isempty(choice)
        fprintf('Empty input. Please enter a valid number.\n');
        continue;
    end
    
    % Convert input to a number
    choice_num = str2double(choice);
    
    % Check if the input is a valid number
    if isnan(choice_num)
        fprintf('Invalid input. Please enter a number.\n');
        continue;
    end

    % Exit condition
    if choice_num == length(files) + 1
        fprintf('Exiting...\n');
        break;
    end
    
    % Validate the choice
    if choice_num < 1 || choice_num > length(files)
        fprintf('Invalid choice. Please enter a number between 1 and %d.\n', length(files) + 1);
        continue;
    end

    % Run the chosen file
    selected_file = files{choice_num};
    fprintf('Running %s...\n', selected_file);
    
    % Execute the chosen file
    try
        run(selected_file);
    catch err
        fprintf('Error running %s: %s\n', selected_file, err.message);
    end
end
