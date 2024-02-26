function robot_dynamics_gui
    % Create a figure and axes for plotting
    fig = figure('Name', 'Robot Dynamics GUI', 'Position', [200, 200, 800, 600]);
    ax = axes('Parent', fig, 'Position', [0.1, 0.3, 0.8, 0.6]);
    
    % Define initial constants
    gear_ratio = 1; % Initial value
    pulley_diameter = 1; % Initial value
    
    % Create sliders for tuning constants
    gearSlider = uicontrol('Style', 'slider', 'Min', 0, 'Max', 10, ...
        'Value', gear_ratio, 'Position', [150, 200, 300, 20], 'Callback', @updatePlot);
    pulleySlider = uicontrol('Style', 'slider', 'Min', 0, 'Max', 10, ...
        'Value', pulley_diameter, 'Position', [150, 170, 300, 20], 'Callback', @updatePlot);

    % Labels for sliders
    uicontrol('Style', 'text', 'String', 'Gear Ratio', 'Position', [70, 200, 70, 20]);
    uicontrol('Style', 'text', 'String', 'Pulley Diameter', 'Position', [70, 170, 70, 20]);

    % Labels for sliders
    gearLabel = uicontrol('Style', 'text', 'String', ['Gear Ratio: ' num2str(gear_ratio)], ...
        'Position', [60, 50, 70, 20]);
    pulleyLabel = uicontrol('Style', 'text', 'String', ['Pulley Diameter: ' num2str(pulley_diameter)], ...
        'Position', [60, 20, 70, 20]);

    % Dropdown menu for selecting a mode
    modeOptions = {'Mode 1', 'Mode 2', 'Mode 3'};
    modeMenu = uicontrol('Style', 'popupmenu', 'String', modeOptions, ...
        'Position', [150, 130, 200, 20], 'Callback', @updatePlot);

    % Checkbox for enabling additional feature
    enableCheckbox = uicontrol('Style', 'checkbox', 'String', 'Enable Feature', ...
        'Position', [150, 100, 150, 20], 'Callback', @updatePlot);

    % Function to update plot
    function updatePlot(~, ~)
        % Get current values from sliders
        gear_ratio = get(gearSlider, 'Value');
        pulley_diameter = get(pulleySlider, 'Value');
        
        % Get selected mode from dropdown menu
        selectedModeIndex = get(modeMenu, 'Value');
        selectedMode = modeOptions{selectedModeIndex};
        
        % Check the state of the checkbox
        enableFeature = get(enableCheckbox, 'Value');
        
        set(gearLabel, 'String', ['Gear Ratio: ' num2str(gear_ratio)]);
        set(pulleyLabel, 'String', ['Pulley Diameter: ' num2str(pulley_diameter)]);


        % Plot robot dynamics based on the updated constants and options
        % Example: Plot some function of the constants and selected mode
        % Here you should replace this with your actual robot dynamics calculations
        x = linspace(0, 10, 1000);
        if strcmp(selectedMode, 'Mode 1')
            y = sin(gear_ratio * x) .* cos(pulley_diameter * x);
        elseif strcmp(selectedMode, 'Mode 2')
            y = cos(gear_ratio * x) .* sin(pulley_diameter * x);
        else
            y = x .* (gear_ratio + pulley_diameter);
        end
        
        % Apply additional feature if checkbox is checked
        if enableFeature
            y = y + 0.5 * sin(2 * x);
        end
        
        % Update the plot
        plot(ax, x, y);
        title(ax, 'Robot Dynamics');
        xlabel(ax, 'Time');
        ylabel(ax, 'Position');
        drawnow; % Force plot update
    end
end