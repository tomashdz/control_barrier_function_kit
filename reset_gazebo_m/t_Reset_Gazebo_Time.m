function cmd_reset_time = t_Reset_Gazebo_Time(logFile, MASTER_IP, ROSFolder_d)
%Reset gazebo world sim time command

    cmd_reset_time = ['export ROS_IP=' MASTER_IP ';' ...
        ' export ROS_MASTER_URI=http://' MASTER_IP ':11311;' ...     % Export the ROS_MASTER_URI
        ' export DISPLAY=:0;' ...
        ' source ' ROSFolder_d 'catkin_ws/devel/setup.bash;' ...        % Source the setup.bash file we determined above
        ' gz world -t&>' logFile ' &']; % Run roscore and pipe output into log file

end

