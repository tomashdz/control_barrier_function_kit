disp('--- sim_terminate.m running');
tic

%% Reset MODE selection
% To release memory of "gzserver", Gazebo is killed at regular interval.(RELEASE_CYCLE)
RELEASE_CYCLE = 20;

% To release memory of "ROS_execute_thread", all is restarted at regular interval.(RESTART_CYCLE)
RESTART_CYCLE = RELEASE_CYCLE; % Now, this is not used!

if exist('falsif_pb', 'var')
    disp(['Finished Count: ' num2str(falsif_pb.nb_obj_eval+1)]);
    if rem(falsif_pb.nb_obj_eval+1, RESTART_CYCLE)==0
        RESET_MODE = 5;
    elseif  rem(falsif_pb.nb_obj_eval+1, RELEASE_CYCLE)==0
        RESET_MODE = 5;
    else
        RESET_MODE = 1;
    end
else
    % 1st trial
    RESET_MODE = 1;
end

%% Save rosbag
%
models_list = getSpawnedModels(gazebo);
hk_idx = find(contains(models_list, 'hk'));

for ihk=hk_idx(1:NUM_ROBOT_rosbag)'
    logFile0 = ['/home/local-admi/Work/20200302_new_map1/LogFile/Stop_',models_list{ihk},'_',datestr(now,'yyyymmdd_HHMMSS'),'.log']; 
    cmd_rosbag_sp_hk = t_Stop_RosBag(logFile0, MASTER_IP, d.ROSFolder, ihk, models_list);
    d.system(cmd_rosbag_sp_hk);
    pause(1.0);
end

pause(10); % Wait for making log files.
%}

%% Reset Function
if RESET_MODE == 1
    try
        %----- pause gazebo -----
        pauseSim(gazebo);
        
        %----- remove Obs + Robots -----
        % Set target model index
        models_list = getSpawnedModels(gazebo);
%         pause(1);
        Obs_idx = find(contains(models_list, 'Obs'));
        hk_idx = find(contains(models_list, 'hk'));
        rem_idx = [Obs_idx' hk_idx'];
        
        % Remove models
        for i = rem_idx
            disp(models_list{i});
            %removeModel(gazebo, models_list{i}); %[!] Error: MATLAB function is sometimes not working ...
            cmd_rm_model = t_Remove_Model(logFile, MASTER_IP, d.ROSFolder, i, models_list); % Run roscore and pipe output into log file
            d.system(cmd_rm_model);
%             pause(1);
            % wait for removing...
            while 1
                models_list_check = getSpawnedModels(gazebo);
                rem_flg = ~find(contains(models_list_check, models_list{i}));
                if isempty(rem_flg)
                    break;
                end
            end
        end
        
        %----- reset ros time -----
        cmd_reset_time =  t_Reset_Gazebo_Time(logFile, MASTER_IP, d.ROSFolder);
        d.system(cmd_reset_time);
        
        %----- kill rostopic node -----
        %
        killed_topic_name = 'rostopic';
        if contains(d.system('ps'), killed_topic_name)
            d.system(['pkill -KILL -f ' killed_topic_name]);
        end
        %}
        
        %----- kill robot node -----
        % Set the target node index
        disp('kill robot nodes: start');
        rosnode_list = rosnode("list");
        hk_idx = find(contains(rosnode_list, 'hk'));
        
        for i = hk_idx
            kill_node = rosnode_list{i};
            cmd_kl = t_Kill_ROS_node(logFile, MASTER_IP, d.ROSFolder, kill_node);  
            d.system(cmd_kl);
        end
        disp('kill robot nodes: end');
    catch
        % If time-out error occurs, then RESET ALL.
        RESET_MODE = 5;
    end
else
    disp('sim_terminate.m: else!')
end

if RESET_MODE==5
    try
    %----- kill all -----
    cmd = ['source ' d.ROSFolder 'catkin_ws/devel/setup.bash; pkill gzserver; pkill roslaunch'];
    d.system(cmd);
    disconnect_from_ROS;
%     pause(5);
    pause(10); % 20200522
    
    %----- restart all -----
    cmd_gazebo = t_Open_Gazebo_World(logFile, MASTER_IP, d.ROSFolder, gazebo_world);
    cmd_rviz = t_Open_Rviz(logFile, MASTER_IP, d.ROSFolder);
       
    d.system(cmd_gazebo);
%     pause(1);
    pause(5); % 20200522
    d.system(cmd_rviz);
    pause(5);
    
    %----- reset ros time -----
    cmd_reset_time = t_Reset_Gazebo_Time(logFile, MASTER_IP, d.ROSFolder);
    d.system(cmd_reset_time);
    
    rosinit(MASTER_IP,'NodeHost',NODE_IP);
%     pause(1);
    pause(5); % 20200522
    gazebo = ExampleHelperGazeboCommunicator();
    pause(5); % 20200522
    
    %----- purge ROS log -----
    disp('Execute "rosclean"');
    cmd_cln = t_RosClean(MASTER_IP,d.ROSFolder);
    d.system(cmd_cln);
    catch
        disp('Restart failed.');
        puase;
    end
else
    disp('sim_terminate.m: else!')
end
toc