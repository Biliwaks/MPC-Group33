function Plot_Xf(Xf,dim,t)

if dim == 4
    figure;
  
    subplot(2,2,1)
    Xf.projection(1:2).plot();
    title('Projection 1:2')

    subplot(2,2,2)
    Xf.projection(2:3).plot();
    title('Projection 2:3')

    subplot(2,2,3)
    Xf.projection(3:4).plot();
    title('Projection 3:4')
    
    sgtitle(t)
    
elseif dim == 2
    figure;
    plot(Xf, 'b');  
    title(t)
else
    fprintf("Not possible to plot with this dimension")
end

end

