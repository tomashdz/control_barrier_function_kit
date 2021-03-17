% This version tries to reduce rapid changes in the input while computing
% optimal risk-bounded controls that lead the system to a goal set 
clear ;
clc;
close all;
%% Initialize
randn('state',4);

% Plotting/video saving options
plotit = 1;
SaveVideo = 0;

% Others:
U = [0 2 ; -pi/6 pi/6];   % v>=0, but angular velocity can be negative
T = 1;  % Time Horizon
SimTime = 25;  % Maximum Simulation Time
u1d = 1.5; % desired speed
% Initial conditions:
x_r = [0 ; 0; 0]; % ego car
x_o = {[-7;1;u1d+0.1*randn],[-5;1;u1d+0.1*randn],[-1; 1;u1d+0.1*randn] ,[2.5;1;u1d+0.1*randn],[5.5;1;u1d+0.1*randn] ,[-10;2;u1d+0.1*randn],  [-7;2;u1d+0.1*randn], [-1;2;u1d+0.1*randn], [0.45;2;u1d+0.1*randn], [5;2;u1d+0.1*randn], [-2;0;u1d+0.1*randn], [3;0;u1d+0.1*randn], [-.5; 3;u1d+0.1*randn], [3;3;u1d+0.1*randn],[-4;3;u1d+0.1*randn]}; %traffic participants

%% Ego and trrafic participant models

x_r_s = sym('x_r',[1,3]);  %x_o, y_o, theta
x_o_s = sym('x_o',[1,3]);  %x_o, y_o, v_o
P_s = sym('p',[3,3]);
err = sym('err');
u_s = sym('u', [1,2]);
% Transformed Bicycle model (Ego, Eq (29))
l = 0.01;
Real_x_r = @(x_r) x_r-l*[cos(x_r(3)); sin(x_r(3)); 0];

f_r = [0 ;0 ; 0];
f_o = @(x_o,x_r) [x_o(3); 0; u1d+(x_o(1)-x_r(1))*exp(-10*((x_o(1)-x_r(1))^2+(x_o(2)-x_r(2))^2))-x_o(3)];
g_r = @(x_r) [cos(x_r(3)) -l*sin(x_r(3)); sin(x_r(3)) l*cos(x_r(3)); 0 1]; 
F_r = @(x,u) f_r + g_r(x)* [u(1); u(2)];
g_o = [0 ;0 ; 0];
F_o = @(x_o,x_r) f_o(x_o,x_r);
F_r_s = F_r(x_r_s, u_s);
F_o_s = F_o(x_o_s,x_r_s);


C = eye(length(x_r_s));
% C = C(1:end-1,:);

G = 0.1*eye(3);
D = 0.1*eye(size(C,2));
R = D*D';
Q = G*G'; 


x_o_hat = x_o;
for i = 1:length(x_o)
    P{i} = eye(3); % riccati states
    x_o_hat{i}(end) = u1d;
    y{i} = C*x_o_hat{i};
end

CR = C'/R;
K_s = P_s*CR;
K = @(P) P*CR; 

A_s = jacobian(F_o_s, x_o_s);
% dP_s = A*P_s+P_s*A'+Q-K_s*C*P_s;              % Danil: Identity might be small for NL
% dP_s = A*P_s*A'+Q
% dp = @(p) dP(p)

%% CBFs and Goal set

p = 0.1*ones(length(x_o),1);  % desired risks
pe = p/2;
err0 = 0.1;
err = [];
for i =1:length(x_o)
    Pmax = norm(P{i}); Pmin = .1;
    err{i} = sqrt(Pmax*err0^2/(Pmin*pe(i)));
end

GoalCenter = [10 ; 3];
r = 0.5*ones(length(x_o),1);   % same radius for all obstacles
rG = 0.1^2;
gamma = 5;  % CBF constant 
Unsafes = cell(length(x_o),1);
% The following loop froms the functions needed for formulating the CBF constarins 
for i = 1:length(Unsafes)
    Unsafes{i} = @(x_r, x_o) (x_r(1)-x_o(1))^2+(x_r(2)-x_o(2))^2-(r(i)+l)^2;
    CBF{i} = @(x_r,x_o) exp(-gamma* Unsafes{i}(x_r,x_o));
    CBFder = vpa(jacobian(CBF{i}(x_r_s,x_o_s),[x_r_s x_o_s]));
    CBF2der = vpa(jacobian(CBFder(length(x_r_s)+1:end), x_o_s));
    ConstCond = vpa(CBFder*[f_r; F_o(x_o_s, x_r_s)]+ 0.5* trace( D'*K_s'*CBF2der*K_s*D)+err{i}*norm(CBFder(length(x_r_s)+1:end)*K_s*C) );
    multCond = vpa(CBFder*[g_r(x_r_s)*u_s'; g_o]);
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
    matchPattern = sprintf('%s(\\d+)_(\\d+)','p');
    replacePattern = sprintf('%s\\($1,$1\\)','p');
    str1 = regexprep(str1,matchPattern,replacePattern);
    ConstCondFun{i} = str2func(['@(x_r,x_o,p,err) ' str1]);
    multCondFun{i} = str2func(['@(x_r,x_o,u) ' str2]); 
    str = '[';
    for j = 1:size(A_s,1)
        for k = 1:size(A_s,2)
            strA{j,k} = char(A_s(j,k));
            matchPattern = sprintf('%s(\\d+)', 'x_r' );
            replacePattern = sprintf('%s\\($1\\)','x_r');
            strA{j,k} = regexprep(strA{j,k},matchPattern,replacePattern);
            matchPattern = sprintf('%s(\\d+)','x_o');
            replacePattern = sprintf('%s\\($1\\)','x_o');
            strA{j,k} = regexprep(strA{j,k},matchPattern,replacePattern); 
            str = strcat(str, strA{j,k});
            if k<size(A_s,2)
                str = strcat(str, ', ');
            elseif j<size(A_s,1)
                str = strcat(str, '; ');
            end
        end
    end
    str = strcat(str, ']');
    Afunc = str2func(['@(x_r,x_o) ' str]);   
end
 
if plotit
    Road.Lanes{1} = @(x_r)(x_r(2)+0.5);
    Road.Lanes{2} = @(x_r)(x_r(2)-3.5);
    Road.midLanes{1} = @(x_r) x_r(2)-0.5;
    Road.midLanes{2} = @(x_r) x_r(2)-1.5;
    Road.midLanes{3} = @(x_r)x_r(2)-2.5;
end
% Goal set and corresponding lyapunov function computation:
Goal = @(x_r)(x_r(2)-GoalCenter(2))^2-rG;
Goalder = vpa(jacobian(Goal(x_r_s(1:2)),x_r_s));
LyapCond = vpa(Goalder*F_r_s);
str1 = char(LyapCond);
matchPattern = sprintf('%s(\\d+)', 'x_r' );
replacePattern = sprintf('%s\\($1\\)','x_r');
str1 = regexprep(str1,matchPattern,replacePattern);
matchPattern = sprintf('%s(\\d+)','u');
replacePattern = sprintf('%s\\($1\\)','u');
str1 = regexprep(str1,matchPattern,replacePattern);
GoalderFunc = str2func(['@(x_r,u) ' str1]);

%% System params

dt = 0.01;
N = SimTime/dt;
for i = 1:length(x_o)
    dW{i} = sqrt(dt)*randn(length(x_o_s),N);   % increments
    dv{i} = sqrt(dt)*randn(size(C,1),N);
end

%% Quadratic Programs

i = 0;
UnsafeRadius = -2;

curr_xr = x_r;
curr_xo = x_o;
curr_xohat = x_o_hat;
curr_P = P;

if plotit
    figure(1)
    axis([-0.5 25 -0.55 3.55])
    set(gcf,'Position',[200 300 1500 300])
    hold on;
    plotReachAvoidSets(Goal,Road,x_r_s)
end
uq = [];
fid = 0;
UnsafeLists = [];

while Goal(curr_xr)>-rG+l^2 && i<N  % Solve the quadratic programs (QP) at each iteration to find an optimal risk bounded control until reaching the goal set    
    
    i = i+1;
    UnsafeList = [];
    Dist = [];
    options =  optimset('Display','off','MaxIter', 2000);
    for j = 1:length(x_o)
        Dists(j) = Unsafes{j}(curr_xr,  curr_xohat{j});
        if Dists(j)<UnsafeRadius  % Consider the obstacles that are in an unsafe radius in your QP 
            UnsafeList = [UnsafeList ,j];
        end
    end
    ai = 1; % Consiedring Eq (17) in the paper as the bound on ai and bi for risk, Let's fix ai
    A = [];
    b =[];
    H = [];
    ff = [];
    for j = 1:length(UnsafeList)
        % CBF Constraints (Eq (15))
        A(2*j-1,[1:length(u_s) length(u_s)+j]) = [multCondFun{UnsafeList(j)}(curr_xr,  curr_xohat{UnsafeList(j)},[1 0]) multCondFun{UnsafeList(j)}(curr_xr,  curr_xohat{UnsafeList(j)},[0 1]) -1]; % multiplier of u , bi
        b(2*j-1) = -ai* CBF{UnsafeList(j)}(curr_xr,  curr_xohat{UnsafeList(j)})- ConstCondFun{UnsafeList(j)}(curr_xr, curr_xohat{UnsafeList(j)}, P{UnsafeList(j)}(:,:,i), err{UnsafeList(j)}(i));
        % Constraints on bi to satisfy pi risk  (Eq(17))
        A(2*j,length(u_s)+j) = 1;
        b(2*j) = min(ai, -1/T*log((1-p(UnsafeList(j)))/(1-CBF{UnsafeList(j)}(curr_xr,  curr_xohat{UnsafeList(j)}))));
    end
    
    % Adding U constraint
    A(end+1,1) = 1; b(end+1) = U(1,2);
    A(end+1,1) = -1;  b(end+1) = -U(1,1);
    A(end+1,2) = 1; b(end+1) = U(2,2);
    A(end+1,2) = -1; b(end+1) = -U(2,1);
    
    % Adding Goal based Lyapunov (Eq (23) or (24))  !! Needs to be changed for a different example
    A(end+1,1:2) = [GoalderFunc(curr_xr,[1 0]) GoalderFunc(curr_xr,[0 1])] ;
    A(end,end+1) = -1;
    b(end+1) = 0;
    A(end+1,end) = 1;
    b(end+1) = eps;
    
    H = [zeros(length(u_s)+length(UnsafeList)+1)];  % u1, u2 , bi for obstacles, delta (for lyapunov)
    H(1,1) = 10;
    H(2,2) = 0.5;
    
    ff = zeros(length(u_s),1);
    ff(length(u_s)+1:length(u_s)+length(UnsafeList)) = 100;
    % u1 close to u1d is prefered but we don't want to change u1 too fast:
    if i >1
        if uq(1)>u1d
            ui = max(u1d,uq(1)-0.1);
        else
            ui = min(u1d,uq(1)+0.1);
        end
    else
        ui = u1d;
    end
    ff(1) = -10*ui;
    ff(2) = 0.5*0.1*curr_xr(3);
    ff(end+1) = 1;
    
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
                warning(output.message)
        end
        if isempty(uq)
            error('No feasible solutions were found. Consider changing design parameters.')
        end
    end
    if length(uq)>3
        [bmax(i), j] = max(uq(3:end-1));
        r = [];
        for k = 1:length(uq)-3
            r(k) = max(0,1-(1-CBF{UnsafeList(k)}(curr_xr,  x_o{UnsafeList(k)}(:,i)))*exp(-uq(k+2)*T));
        end
        risk(i) = max(r(k));
    else
        bmax(i) = 0;
        risk(i) = 0;
    end
    minDist(i) = min(Dists);
    delta(i) = uq(end); % Lyapunov slack variable
    curr_u = uq(1:length(u_s)); % Control actions
    uq_b = uq;
    u_r(:,i) = curr_u;
    
    
    % actual x's, estimates
    for j = 1:length(x_o)
        curr_xo{j} = curr_xo{j} + F_o(curr_xo{j},curr_xr)*dt + G*dW{j}(:,i);
        x_o{j}(:,i+1) = curr_xo{j};
        
        x_o_hat_pred = curr_xohat{j} + F_o(curr_xohat{j},curr_xr)*dt ;
        At = Afunc(curr_xr,x_o_hat_pred);
        P_pred = (eye(3)+At*dt)*curr_P{j}*(eye(3)+At*dt)'+Q; 
        
        dy = C*curr_xo{j}*dt + D*dv{j}(:,i);
        y{j}(:,i+1) = y{j}(:,i) + dy;
        ythild = y{j}(:,i+1) - C*x_o_hat_pred;
        S = C*P_pred*C' + R;
        Kt = P_pred*C'/S;
        
        curr_xohat{j} = x_o_hat_pred + Kt*ythild; 
        x_o_hat{j}(:,i+1) = curr_xohat{j};
        
        curr_P{j} = P_pred - Kt*C*P_pred; 
        P{j}(:,:,i+1) = curr_P{j};
        
    end
    curr_xr = curr_xr + F_r(curr_xr,curr_u)*dt;
    x_r(:,i+1) = curr_xr;
    
    % error update    
    for ii =1:length(x_o)
        Pmax = norm(curr_P{ii}); Pmin = .1;
        err{ii}(i+1) = sqrt(Pmax*err0^2/(Pmin*pe(ii)));
    end
    
    
    r_x_r(:,i+1) = Real_x_r(curr_xr);  % States in original coordiantes 
    % Plotting
    if plotit && rem(i,10)==1
        if i>1
            set(rob,'Visible','off')
            for io = 1:length(x_o)
                set(ob{io},'Visible','off')
            end
        end
        args = {'LineWidth', 2};
        rob = plot(r_x_r(1,i),r_x_r(2,i),'o','color','b','markerfacecolor', 'b', args{:});
        for io = 1:length(x_o)
            ob{io} = plot(x_o{io}(1,i),x_o{io}(2,i),'o','color','r','markerfacecolor', 'r', args{:});
        end
        fid = fid+1;
        F(fid) = getframe;
        drawnow
    end
end
%% Post Processing

% Plotting 
figure(1); plot(r_x_r(1,:),r_x_r(2,:),'o','color','b',args{:})
for j = 1:100:i
    text(r_x_r(1,j),r_x_r(2,j)+0.06,num2str((j-1)*dt),'color','w')
end
figure(2);ax = subplot(4,1,1);plot(0:dt:(i-1)*dt, u_r(1,:)); ylabel('u(1)','interpreter','latex','fontsize',14); xlim([0,16.2]); grid on;ax.FontSize = 10; 
figure(2);ax = subplot(4,1,2);plot(0:dt:(i-1)*dt, u_r(2,:)); ylabel('u(2)','interpreter','latex','fontsize',14); xlim([0,16.2]);grid on;ax.FontSize = 10; 
figure(2);ax = subplot(4,1,3);plot(0:dt:(i-1)*dt, risk); ylabel('$\max_i(\bar p_i^B)$','interpreter','latex','fontsize',14); xlim([0,16.2]);grid on;ax.FontSize = 10; 
figure(2);ax = subplot(4,1,4); plot(0:dt:(i-1)*dt, minDist); ylabel('$\min_i(h_i(\tilde{x}_i))$','interpreter','latex','fontsize',14); xlim([0,16.2]);xlabel('time (s)','interpreter','latex','fontsize',12);grid on; ax.FontSize = 10; 

% Video saving
if SaveVideo
    writerObj = VideoWriter('test3.avi'); %Attempt to create an avi
    writerObj.FrameRate = 10;
    open(writerObj);
    for t= 1:fid
        
        writeVideo(writerObj,F(t))
        
    end
    close(writerObj);
end


%% Functions

function plotReachAvoidSets(Goal,Road,x_r_s)
args = {'-black', 'LineWidth', 2};
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
end