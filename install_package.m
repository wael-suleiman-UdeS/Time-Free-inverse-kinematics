% Define the package name
pkg_name = "optim";

% Check if the package is already installed
installed_packages = pkg("list");

is_installed = false;
for i = 1:length(installed_packages)
    if strcmp(installed_packages{i}.name, pkg_name)
        is_installed = true;
        break;
    end
end

% Install the package if not already installed
if ~is_installed
    fprintf('Package "%s" is not installed. Installing...\n', pkg_name);
    try
        pkg("install", "-forge", pkg_name);
        pkg("load", pkg_name); % Load the package after installation
        fprintf('Package "%s" installed and loaded successfully.\n', pkg_name);
    catch err
        fprintf('Failed to install package "%s". Error: %s\n', pkg_name, err.message);
    end
else
    fprintf('Package "%s" is already installed.\n', pkg_name);
    pkg("load", pkg_name); % Ensure the package is loaded
    fprintf('Package "%s" is loaded.\n', pkg_name);
end
