function runRC(controlfcn)

    node = ros2node("rc_control_matlab");
    disp("ROS Node created.")

    latest_odom = ros2message("nav_msgs/Odometry");
    ros2subscriber(node, "/odom", "nav_msgs/Odometry", updateOdom);

    latest_map = ros2message("nav_msgs/OccupancyGrid");
    ros2subscriber(node, "/map", "nav_msgs/OccupancyGrid", updateMap);

    command_pub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");

    rate = ros2rate(obj.Node, 20);

    disp("Starting control loop")
    reset(rate)
    while (1)
        control = controlfcn(latest_odom, latest_map);
        sendCommand(control)
        waitfor(rate)
    end

    function updateOdom(msg)
        latest_odom = msg;
    end

    function updateMap(msg)
        latest_map = msg;
    end

    function sendCommand(msg)
        send(command_pub, msg)
    end
end