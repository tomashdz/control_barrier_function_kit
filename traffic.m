clear ;
% clc;
close all;
x_r = [0 ; 0; 0];
x_o = {[-1.5; 1], [3;1] ,[-5;1],[-15;1], [-10;2], [-4;2], [0;2],[7;2]}; 
% x_o = {[-1; 1], [3;1] [-5;2], [0;2],[7;2],}; 

t = sym('t');
x_r_s = sym('x_r',[1,3]);
x_o_s = sym('x_o',[1,2]);
u_s = sym('u', [1,2]);

U = [0 2 ; -pi/6 pi/6];   % v_x>=0, but v_y can be negative: more limited to better represent steering
T = 0.5;
SimTime = 25;
plotit = 1;
plotlanes = 1;
% Point mass model
l = 0.01;
Real_x_r = @(x_r) x_r-l*[cos(x_r(3)); sin(x_r(3)); 0];
f = @(x)  [0 ;0; 0]; 
g = @(x) [cos(x(3)) -l*sin(x(3)); sin(x(3)) l*cos(x(3)); 0 1];
f_r = @(t,x,u) f(x)+g(x)* [u(1); u(2)];
F_r = f_r(0, x_r_s, u_s);
u1d = 1.2;
max_u2d = pi/20;
% Obstacles move on a line with varying velocity 
f_o = @(x) [1.5;0]; 
g_o = @(x) [0.3; 0];
% f_o = @(t,x,w) f(x)+g(x)*w;
% F_o = f_o(0, x_o_s, 0);
% w(i) = 0;

%% CBFs and Goal set
p = 0.1 *ones(length(x_o),1);  % desired risks
GoalCenter = [10 ; 3];
r = 0.4^2*ones(length(x_o),1);   % same radius for all circles
rG = 0.1^2;
rLane = 0.5^2;
gamma = 20;
Unsafes = cell(length(x_o),1);
for i = 1:length(Unsafes)
    Unsafes{i} = @(x_r, x_o) (x_r(1)-x_o(1))^2+(x_r(2)-x_o(2))^2-r(i)-l;
    CBF{i} = @(x_r,x_o) exp(-gamma* Unsafes{i}(x_r,x_o));
    CBFder = vpa(jacobian(CBF{i}(x_r_s(1:2),x_o_s(1:2)),[x_r_s x_o_s]));
    CBF2der = vpa(jacobian(CBFder(length(x_r_s)+1:end),x_o_s));
    ConstCond = vpa(CBFder*[f(x_r_s); f_o(x_o_s)]+ 0.5* trace( g_o(x_o_s)'*CBF2der*g_o(x_o_s)) );
    multCond = vpa(CBFder*[g(x_r_s)*u_s'; zeros(length(x_o_s),1)]);
    str1 = char(ConstCond);
    str2 = char(multCond);
    matchPattern = sprintf('%s(\\d+)', 'x_r' );
    replacePattern = sprintf('%s\\($1\\)','x_r');
    str1 = regexprep(str1,matchPattern,replacePattern);
    str2 = regexprep(str2,matchPattern,replacePattern);
    matchPattern = sprintf('%s(\\d+)','x_o');
    replacePattern = sprintf('%s\\($1\\)','x_o');
    str1 = regexprep(str1,matchPattern,replacePattern);
    str2 = regexprep(str2,matchPattern,replacePattern);
    matchPattern = sprintf('%s(\\d+)','u');
    replacePattern = sprintf('%s\\($1\\)','u');
    str2 = regexprep(str2,matchPattern,replacePattern);
    ConstCondFun{i} = str2func(['@(x_r,x_o) ' str1]);
    multCondFun{i} = str2func(['@(x_r,x_o,u) ' str2]);
 end
Goal = @(x_r)(x_r(2)-GoalCenter(2))^2-rG;
Road.Lanes{1} = @(x_r)(x_r(2)-2)^2-rLane;
Road.Lanes{2} = @(x_r)(x_r(2)-1)^2-rLane;
Road.Lanes{3} = @(x_r)(x_r(2)-0)^2-rLane;
Road.midLanes{1} = @(x_r)x_r(2);
Road.midLanes{2} = @(x_r) x_r(2)-1;
Road.midLanes{3} = @(x_r)x_r(2)-2;
Road.midLanes{4} = @(x_r)x_r(2)-3;
Goalder = vpa(jacobian(Goal(x_r_s(1:2)),x_r_s));
LyapCond = vpa(Goalder*F_r);
str1 = char(LyapCond);
matchPattern = sprintf('%s(\\d+)', 'x_r' );
replacePattern = sprintf('%s\\($1\\)','x_r');
str1 = regexprep(str1,matchPattern,replacePattern);
matchPattern = sprintf('%s(\\d+)','u');
replacePattern = sprintf('%s\\($1\\)','u');
str1 = regexprep(str1,matchPattern,replacePattern);
GoalderFunc = str2func(['@(x_r,u) ' str1]);
%% Obstacle's Simulation
% Since obstacles move irrespective of the robot, we
% can simulate them individually in advance

% Wiener Process
randn('state',1);
dt = 0.01;
N = SimTime/dt;
dW = sqrt(dt)*randn(length(x_o),N);   % increments

% We generally need to use the following, but since in this example f_o =0
% g = [1,0], x_o can be computed like above
for j = 1:length(x_o)
    for i = 1:N
        x_o{j}(:,i+1) = x_o{j}(:,i)+ f_o(x_o{j}(:,i))*dt + g_o(x_o{j}(:,i))*dW(j,i)  ;
    end
end

%% Quadratic Programs
i = 0;
UnsafeRadius = 2;
curr_xr = x_r;
x_r_traj = x_r;
t_traj = 0;

figure(1)
if plotit
    figure(1)
    axis([-2 20 -1 3.5])
    set(gcf,'Position',[300 400 1500 300])
    hold on;
    if plotlanes
        plotReachAvoidSets(Goal,Road,x_r_s)
    end
end

fid = 0;
% final pose/ acceleration instead of velocity/ Risk and other related
% measures plot
while Goal(curr_xr)>-0.1^2+l+eps && i<N
    i = i+1;
    UnsafeList = [];
    options =  optimset('Display','off','MaxIter', 2000);
    for j = 1:length(x_o)
        Dists(j) = Unsafes{j}(curr_xr,  x_o{j}(:,i));
        if Dists(j)<UnsafeRadius
            UnsafeList = [UnsafeList ,j];
        end
    end
    % Consiedring (22) as the bound on ai and bi for risk, Let's fix ai
    ai = 1;
    A = [];
    b =[];
    H = [];
    ff = [];
   for j = 1:length(UnsafeList)
        % CBF Constraints
        A(2*j-1,[1:length(u_s) length(u_s)+j]) = [multCondFun{UnsafeList(j)}(curr_xr,  x_o{UnsafeList(j)}(:,i),[1 0]) multCondFun{UnsafeList(j)}(curr_xr,  x_o{UnsafeList(j)}(:,i),[0 1]) -1]; % multiplier of u , bi
        b(2*j-1) = -ai* CBF{UnsafeList(j)}(curr_xr,  x_o{UnsafeList(j)}(:,i))- ConstCondFun{UnsafeList(j)}(curr_xr, x_o{UnsafeList(j)}(:,i));  
        % Constraints on bi to satisfy pi risk
        A(2*j,length(u_s)+j) = 1;
        b(2*j) = min(ai, -1/T*log((1-p(UnsafeList(j)))/(1-CBF{UnsafeList(j)}(curr_xr,  x_o{UnsafeList(j)}(:,i)))));
    end
    
    % Adding the bi's
%     A = [A, -eye(length(x_o)) zeros(length(x_o),1)];
%     A = [A, -eye(length(x_o))];

    % Adding U constraint
    A(end+1,1) = 1; b(end+1) = U(1,2);
    A(end+1,1) = -1;  b(end+1) = -U(1,1);
    A(end+1,2) = 1; b(end+1) = U(2,2);
    A(end+1,2) = -1; b(end+1) = -U(2,1);
    
    H = [zeros(length(u_s)+length(UnsafeList))];  % u1, u2 , b1 to  b4 for obstacles, delta (for lyapunov)
    H(1:length(u_s), 1:length(u_s)) = eye(length(u_s));
    ff(length(u_s)+1:length(u_s)+length(UnsafeList)) = 10^4; %10^8
    % ff(end) = 0.1;
    ff(1) = -u1d;
    ff(2) = -max_u2d; 

    % In order to enforce the pose to become parallel to lane we can change the
    % refrence angle as follows
    if Goal(curr_xr)<0.5^2
        ff(2) = min(curr_xr(3),U(2,2));
    end


    [uq,fval,exitflag,output]= quadprog(H, ff, A, b,[],[],[],[],[],options);
    if ~(exitflag==1)
        switch exitflag
            case 0
                warning('exitflag 0: Stopped cause of max iteration')
            case -2
                warning('NO feasible solution')
            case -3
                warning('exitflag -3: Unbounded')
            otherwise
                warning(['exitflag', num2str(exitflag) ])
        end
    end
    if length(uq)>2
        [bmax(i), j] = max(uq(3:end));
        r = [];
        for k = 1:length(uq)-2
            r(k) = max(0,1-(1-CBF{UnsafeList(k)}(curr_xr,  x_o{UnsafeList(k)}(:,i)))*exp(-uq(k+2)*T));
        end
        risk(i) = max(r(k));
    else 
        risk(i) = 0;
        bmax(i) = 0;
    end
    minDist(i) = min(Dists);
    curr_u = uq(1:length(u_s));
    uq_b = uq;
    u_r(:,i) = curr_u;
    Cl_fr = @(t,x) f_r(t,x,curr_u);
    [T_traj, nextStates] = ode45(Cl_fr, [(i-1)*dt i*dt], curr_xr);
    curr_xr = nextStates(end,:)';
    x_r(:,i+1) = curr_xr;
    x_r_traj = [x_r_traj nextStates'];
    t_traj = [t_traj T_traj'];
    
    
    if plotit && rem(i,15)==1
        if i>1
            set(rob,'Visible','off')
            for io = 1:length(x_o)
                set(ob{io},'Visible','off')
            end
        end
        args = {'LineWidth', 2};
        r_x_r = Real_x_r(x_r);
        rob = plot(r_x_r(1,i),r_x_r(2,i),'o','color','b',args{:});
        for io = 1:length(x_o)
            ob{io} = plot(x_o{io}(1,i),x_o{io}(2,i),'o','color','r',args{:});
        end
    %     figure(2)
    %     hold on;
    %     plot(i,u_r(i),'o')
        fid = fid+1;
        F(fid) = getframe;
        drawnow 
    end
end
figure(1); plot(x_r(1,:),x_r(2,:),'o','color','b',args{:})
% writerObj = VideoWriter('test2.avi'); %Attempt to create an avi
% writerObj.FrameRate = 10;
% open(writerObj);
% for t= 1:fid
% 
%     writeVideo(writerObj,F(t))
% 
% end
% 
% close(writerObj);
figure(2);subplot(4,1,1);plot(0:dt:(i-1)*dt, u_r); ylabel('Input')
figure(2);subplot(4,1,2);plot(0:dt:(i-1)*dt, risk); ylabel('$\max(\bar p_i^B)$','interpreter','latex','fontsize',14)
subplot(4,1,3);plot(0:dt:(i-1)*dt, minDist);ylabel('$\min(h(x))$','interpreter','latex','fontsize',14)
subplot(4,1,4); plot(1:i,bmax); xlabel('time'); ylabel('$\max(b_i)$','interpreter','latex','fontsize',14)

 function plotReachAvoidSets(Goal,Road,x_r_s)
             args = {'-black', 'LineWidth', 2};
%             prefix = this.plant.statePrefix;
             matchPattern = sprintf('%s(\\d*)\\^(\\d*)','x_r');
             replacePattern = sprintf('%s$1\\.\\^$2', 'x_r');
            for ij = 1:length(Road.Lanes)
                fun = char(expand(vpa((Road.Lanes{ij}(x_r_s)))));
                str = regexprep(fun,matchPattern,replacePattern);
                str = sprintf('@(%s1,%s2) %s', 'x_r', 'x_r', str);          
                fimplicit(str2func(str), args{:},'MeshDensity',100);
                hold on
            end
            args = {'--black', 'LineWidth', 2};
%             prefix = this.plant.statePrefix;
             matchPattern = sprintf('%s(\\d*)\\^(\\d*)','x_r');
             replacePattern = sprintf('%s$1\\.\\^$2', 'x_r');
            for ij = 1:length(Road.midLanes)
                fun = char(expand(vpa((Road.midLanes{ij}(x_r_s)))));
                str = regexprep(fun,matchPattern,replacePattern);
                str = sprintf('@(%s1,%s2) %s', 'x_r', 'x_r', str);          
                fimplicit(str2func(str), args{:},'MeshDensity',100);
                hold on
            end
            
            args = {'green-', 'LineWidth', 2};
            fun = char(expand(vpa((Goal(x_r_s)))));
            str = regexprep(fun,matchPattern,replacePattern);
            str = sprintf('@(%s1,%s2) %s', 'x_r', 'x_r', str);          
            fimplicit(str2func(str), args{:},'MeshDensity',100);
%             axis([-1 16 -1 4])
%             drawSet(set, granularity)                          
end