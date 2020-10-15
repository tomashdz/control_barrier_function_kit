function cmd_cln = t_RosClean(MASTER_IP, ROSFolder_d)
%Start Rviz Viewer

    cmd_cln = ['export ROS_IP=' MASTER_IP ';' ...
        ' export ROS_MASTER_URI=http://' MASTER_IP ':11311;' ...     % Export the ROS_MASTER_URI
        ' export DISPLAY=:0;' ...
        ' source ' ROSFolder_d 'catkin_ws/devel/setup.bash;' ...        % Source the setup.bash file we determined above
        'rosclean purge -y &']; % Run roscore and pipe output into log file
end

