function cmd_rm_model = t_Remove_Model(logFile, MASTER_IP, ROSFolder_d, id, models_list)
%Remove Gazebo Object

    cmd_rm_model = ['export ROS_IP=' MASTER_IP ';' ...
        ' export ROS_MASTER_URI=http://' MASTER_IP ':11311;' ...     % Export the ROS_MASTER_URI
        ' export DISPLAY=:0;' ...
        ' source ' ROSFolder_d 'catkin_ws/devel/setup.bash;' ...        % Source the setup.bash file we determined above
        ' rosservice call gazebo/delete_model "{model_name: ' models_list{id} '}"&>' logFile ' &']; % Run roscore and pipe output into log file
end

