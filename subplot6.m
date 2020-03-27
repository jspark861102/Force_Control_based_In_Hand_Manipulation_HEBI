%% joint
function subplot6(x,y,title_name,ymin,ymax)    
    figure;
    for i = 1 : 6
        subplot(2,3,i)
        plot(x,y(:,i))
        title([title_name num2str(i)])
%         axis([-0.1 10.1 ymin ymax])
        ylim([ymin ymax])
        grid on
    end
end

