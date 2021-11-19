function ros_vars = ros_setup()

    msg_dir = '../../../../src/matlab_msg_gen_ros1/glnxa64/install/m';
    if ~exist(msg_dir, 'dir')
        rosgenmsg('../../');
    else
        addpath(msg_dir);
    end

    ros_vars.node = ros.Node('sim');

    ros_vars.thrust_sub = ...
        ros.Subscriber(ros_vars.node, ...
                        '/alpha/thrust_cmd', ...
                        'geometry_msgs/Point');
    ros_vars.shutdown_sub = ...
        ros.Subscriber(ros_vars.node, ...
                        '~shutdown_sim', ...
                        'std_msgs/Bool');
    ros_vars.odom_pub = ...
        ros.Publisher(ros_vars.node, ...
                       '/alpha/odom', ...
                       'nav_msgs/Odometry');

    ros_vars.tftree = ros.TransformationTree(ros_vars.node);
                   
    ros_vars.thrust_msg = rosmessage('geometry_msgs/PointStamped');
    ros_vars.shutdown_msg = rosmessage('std_msgs/Bool');
    ros_vars.odom_msg = rosmessage('nav_msgs/Odometry');
    ros_vars.tf_msg = rosmessage('geometry_msgs/TransformStamped');

end