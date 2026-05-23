function P = threespline(Pp, radius, circleCenter, start, goal)
    pt = Pp;
    fig = figure(2);
    [x, y, z] = sphere;
    for i = 1:length(radius)
        mesh(radius(i) * x + circleCenter(i, 1), radius(i) * y + circleCenter(i, 2), radius(i) * z + circleCenter(i, 3));
        hold on;
    end
    axis equal;
    grid on;
    axis equal;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    title('曲线平滑后的路径');
    hold on;
    scatter3(start(1),start(2),start(3),'filled','g');
    scatter3(goal(1),goal(2),goal(3),'filled','k');
    
    [row,column] = size(pt);
    n = length(pt);
    k = 3;
    U = zeros(1,n+k+3);
    
    if(column == 2)
        x = pt(:,1);
        y = pt(:,2);
    else
        x = pt(:,1);
        y = pt(:,2);
        z = pt(:,3);
    end

    temp = zeros(1,n-1);
    for i = 1:n-1
        if(column == 2)
            temp(i) = sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2);
        else
            temp(i) = sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2+(z(i+1)-z(i))^2);
        end
    end
    
    sumtemp = sum(temp);
    for i = 1:k+1
        U(i) = 0;
    end
    for i = n+k:n+k+3
        U(i) = 1;
    end
    
    for i = k+1:n+k-2
        U(i+1) = U(i)+temp(i-k)/sumtemp;
    end

    if(column == 2)
        dpt1 = [0 1];
        dptn = [-1 0];
    else
        dpt1 = [0 0 1];
        dptn = [-1 0 0];
    end
    
    dU = zeros(1,n+k+3);
    for i = k+1:n+k-1
        dU(i) = U(i+1)-U(i);
    end

    A = zeros(n);
    if(column == 2)
        E = zeros(n,2);
    else
        E = zeros(n,3);
    end
    
    A(1,1) = 1;
    A(n,n) = 1;
    E(1,:) = pt(1,:)+(dU(4)/3)*dpt1;
    E(n,:) = pt(n,:)-(dU(n+2)/3)*dptn;
    
    for i = 2:n-1
        A(i,i-1) = dU(i+3).^2/(dU(i+1)+dU(i+2)+dU(i+3));
        A(i,i) = dU(i+3)*(dU(i+1)+dU(i+2))/(dU(i+1)+dU(i+2)+dU(i+3))+...
            dU(i+2)*(dU(i+3)+dU(i+4))/(dU(i+2)+dU(i+3)+dU(i+4));
        A(i,i+1) = dU(i+2).^2/(dU(i+2)+dU(i+3)+dU(i+4));
        E(i,:) = (dU(i+2)+dU(i+3))*pt(i,:);
    end
    
    D = A\E;
    D = [pt(1,:);D;pt(n,:)];
    [s,~] = size(D);

    dt = 0.01;
    P = [];
    syms dx;
    
    for i = k+1:s
        u = U;
        d = sym(D);
        for m = 1:k
            for j = i-k:i-m
                alpha(j) = (dx-u(j+m))/(u(j+k+1)-u(j+m));
                d(j,:) = (1-alpha(j))*d(j,:)+alpha(j)*d(j+1,:);
            end
        end
        M = subs(d(i-k,:),dx,(u(i):dt:(u(i+1)-dt))');
        P = [P;double(M)];
    end
    
    M = subs(d(s-k,:),dx,1);
    P = [P;double(M)];
    
    figure(2);
    view(3);
    
    if(column == 2)
        plot(pt(:,1), pt(:,2), '*r');
        hold on;
        plot(D(:,1), D(:,2), 'b-o');
        hold on;
        plot(P(:,1), P(:,2), 'r');
        hold on;
    else
        hold on;
        plot3(P(:,1), P(:,2), P(:,3), 'r', 'LineWidth', 1.5);
        hold on;
    end
    
    axis equal;
    grid on;
end
