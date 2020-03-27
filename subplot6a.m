function subplot6a(x,y,title_name,control_time)    
    figure;
    for i = 1 : 6
        subplot(2,3,i)
        plot(x,y(:,i))
        title([title_name num2str(i)])
        hold on
%         plot([300 300], [min(y(:,i)) max(y(:,i))],'--r')
%         axis([-0.1 10.1 ymin ymax])
%         ylim([min(y(:,i)) max(y(:,i))])
        xlabel('t(sec)')
        grid on
        time_interval = cumsum(control_time(4:end));
        for k = 1 : length(time_interval)
            plot([time_interval(k) time_interval(k)], [min(y(:,i)) max(y(:,i))],'--r')
        end
    end
end