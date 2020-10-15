function cmd_kl = t_Kill_ROS_node(logFile, MASTER_IP, ROSFolder_d, kill_node)
%Kill ROS Node command

    cmd_kl = ['export ROS_IP=' MASTER_IP ';' ...
        ' export ROS_MASTER_URI=http://' MASTER_IP ':11311;' ...     % Export the ROS_MASTER_URI
        ' export DISPLAY=:0;' ...
        ' source ' ROSFolder_d 'catkin_ws/devel/setup.bash;' ...        % Source the setup.bash file we determined above
        ' rosnode kill ' kill_node '&>' logFile ' &']; % Run roscore and pipe output into log file        

end

