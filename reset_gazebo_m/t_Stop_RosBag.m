function cmd_rosbag_sp_hk = t_Stop_RosBag(logFile, MASTER_IP, ROSFolder_d, id, models_list)
%Stop Robots bag file

    cmd_rosbag_sp_hk = ['export ROS_IP=' MASTER_IP ';' ...
        ' export ROS_MASTER_URI=http://' MASTER_IP ':11311;' ...     % Export the ROS_MASTER_URI
        ' export DISPLAY=:0;' ...
        ' source ' ROSFolder_d 'catkin_ws/devel/setup.bash;' ...        % Source the setup.bash file we determined above
        ' rosservice call /' models_list{id} '/stop_record_service "data: true"&>' logFile ' &'];
end

