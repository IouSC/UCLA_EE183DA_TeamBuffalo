function plot_car_state(x)
    % Variables and Constants
    L = 762.5; % Length of the box [mm]
    W = 497.5; % Width of the box [mm]
    
    % Plot Path of the car 
    figure(1);
    hold on;
    plot(x(1,:),x(2,:),'g');
    
    % Set figure
    title('$State\ of\ the\ car\ with\ state\ estimator$','interpreter','latex');
    xlabel('$p_x$','interpreter','latex');
    ylabel('$p_y$','interpreter','latex');
    set(gca,'ticklabelinterpreter','latex');
    axis([0 L 0 W]);
    grid on; box on;
    
    % Plot the car
    C_W = 100;  % width of the car [mm]
    C_L = 97;   % length of the car [mm]
    
    for i = 1: length(x)
        [cor1,cor2,cor3,cor4] = corners(x(:,i));
        % x axis of the car
        cx = [cor1(1);cor2(1);cor3(1);cor4(1);cor1(1)];
        % y axis of the car
        cy = [cor1(2);cor2(2);cor3(2);cor4(2);cor1(2)];
        % Plot car sensorsensor reference (top right of the car)
        pcr = plot(cor1(1),cor1(2),'r*');
        % Plot the Front side of the car 
        pcf = plot(cx(1:2),cy(1:2),'color',[0, 0.25, 0.25]);
        % Plot the rest shape of the car 
        pcs = plot(cx(2:5),cy(2:5),'color',[0.3010, 0.7450, 0.9330]);
        
        % updae on graph
        drawnow;
        refreshdata; 
        % delete previous car stat
        if i ~= length(x)
            delete(pcs);
            delete(pcf);
            delete(pcr);
        end
    end 
    
    % figure legend
    legend({'$Estimated\ state\ path$','$Car\ reference\ point$',...
            '$Front\ of\ the\ car$','$Shape\ of\ the\ car$'},...
            'location','best','interpreter','latex');
    hold off
end