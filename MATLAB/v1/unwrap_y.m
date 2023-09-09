function unwrapped = unwrap_y(x)
    bigness=size(x);
    L_x=bigness(1)*bigness(2);
    unwrapped = sym(zeros(L_x,1));
    for i=1:bigness(1)
        for j=1:bigness(2)
            unwrapped(i+bigness(1)*(j-1)) = x(i,j);
        end
    end
end