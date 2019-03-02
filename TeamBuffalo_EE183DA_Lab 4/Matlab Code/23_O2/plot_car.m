function robot_path_dis = plot_car(x,robot_path_dis)   
    % Note
        % tightfig is an open source function that remove excess margins from figures
        % https://www.mathworks.com/matlabcentral/fileexchange/34055-tightfig-hfig

    % Variables and Constants
    L = 762.5; % Length of the box [mm]
    W = 497.5; % Width of the box [mm]

    % Create a figure to plot the state of the car
    figure(1);
    hold on;
    % Set figure
    title('$RRT^*$-Planned Robot Trajectory','interpreter','latex','fontsize',16);
    xlabel('$p_x$','interpreter','latex');
    ylabel('$p_y$','interpreter','latex');
    set(gca,'ticklabelinterpreter','latex');
    axis([0 L 0 W]);
    box on; tightfig;

    % Car Dimensions
    C_W = 100;  % width of the car [mm]
    C_L = 97;   % length of the car [mm]

    for i = 1:3
        [cor1,cor2,cor3,cor4] = corners(x(:,i));
        % x axis of the car
        cx = [cor1(1);cor2(1);cor3(1);cor4(1);cor1(1)];
        % y axis of the car
        cy = [cor1(2);cor2(2);cor3(2);cor4(2);cor1(2)];
        % Plot RRT* car reference point
        mid(1) = (cor1(1)+cor2(1))/2;
        mid(2) = (cor1(2)+cor2(2))/2;
        pcr = plot( mid(1),mid(2),'r*');
        % Plot the Front side of the car
        pcf = plot(cx(1:2),cy(1:2),'color',[0, 0.25, 0.25]);
        % Plot the rest shape of the car
        pcs = plot(cx(2:5),cy(2:5),'color',[0.3010, 0.7450, 0.9330]);

        % Update on figure
        drawnow;
        refreshdata;
        
        % Figure Legend
        legend({'$RRT^*$ car reference point','Front side of the car','Border of the car'},...
                'interpreter','latex','location','northeast','Autoupdate','off');
        pause(1);
        
        % Delete previous car state
        delete(pcs);
        delete(pcf);
        delete(pcr);
    end

    % Calculate total distance traveled by the robot;
    robot_path_dis = robot_path_dis + pdist([x(1,2) x(2,2); x(1,3) x(2,3);]);
    annotationStr = ['Distance of the path robot taken = ' num2str(robot_path_dis) ' [mm]'];
    annotationStr2 = ['$RRT^*$ Optimal Path'];
    delete(findall(gcf,'type','annotation'));  
    annotation('textbox',[.1 .875 .5 .05],'String',annotationStr,'interpreter','latex',...
               'FitBoxToText','on','EdgeColor','none');
    annotation('line',[.11 .15],[.875 .875],'Color','m','Linewidth',2);
    annotation('textbox',[.15 .845 .5 .05],'String',annotationStr2,'interpreter','latex',...
               'FitBoxToText','on','EdgeColor','none');
    hold off

    % Save final result figure to .png
    savetitleStr = ['TeamBuffalo_EE183DA_Lab4_Test.png'];
    saveas(gcf,savetitleStr);
end
