
%Load jointspace itself by clicking the csv file
A = table2array(jointspace);

dof = 6;
A(:,1) = A(:,1) - A(1,1);
A(:,dof+2) = A(:,dof+2) - A(1,dof+2);

figure(1)
for i = 1:1:dof
    subplot(dof,1,i);
    x = A(:,1);
    y = A(:,i+1);
    plot(x,y, 'LineWidth',2);
    hold on
    x = A(:,2+dof);
    y = A(:,2+dof+i);
    plot(x,y,'LineWidth',2);
  
    if i == 1
    title('Joint Trajectory Tracking')
    end
    
    if i == 3
        title_name = sprintf('q_%i [rad]',i);
        ylabel(title_name)
    else
        title_name = sprintf('q_%i [m]',i);
        ylabel(title_name)
    end
    
    if i == dof
     xlabel('time [s]')
    end
    
end