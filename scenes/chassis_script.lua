function sysCall_init()
    sim = require('sim')
    simROS2 = require('simROS2')
    
    -- 1. GET HANDLES (Update these if your names are different!)
    bodyHandle = sim.getObject("/Chassis")  -- Make sure this points to the CHASSIS, not '.'
    leftMotor = sim.getObject("/LeftMotor")
    rightMotor = sim.getObject("/RightMotor")
    graphHandle = sim.getObject("/graph")

    -- 2. SETUP GRAPH STREAMS
    -- We graph "Tilt" in DEGREES so it is visible next to Velocity (0-60)
    tiltStream = sim.addGraphStream(graphHandle, 'Tilt_Deg', 'deg', 0, {1,0,0}) -- Red
    velStream = sim.addGraphStream(graphHandle, 'Motor_Cmd', 'rad/s', 0, {0,1,0}) -- Green

    -- 3. ROS2 SETUP
    tiltPub = simROS2.createPublisher('/robot_tilt', 'std_msgs/msg/Float32')
    velSub = simROS2.createSubscription('/cmd_vel_wheels', 'std_msgs/msg/Float32', 'setVel_callback')
    
    print("--- Robot Brain Online: Logs & Graphing Active ---")
end

function setVel_callback(msg)
    local targetVel = msg.data
    
    -- 4. MOTOR CONTROL (Fixing the 'Spinning' issue)
    -- Try sending Positive to BOTH. If it moves backward, change BOTH to -targetVel
    sim.setJointTargetVelocity(leftMotor, targetVel)
    sim.setJointTargetVelocity(rightMotor, targetVel) 
    
    -- Update Green Line on Graph
    sim.setGraphStreamValue(graphHandle, velStream, targetVel)
end

function sysCall_sensing()
    -- 1. Get Orientation
    local orient = sim.getObjectOrientation(bodyHandle, -1)
    
    -- 2. CAPTURE ALPHA (The correct axis!)
    local alpha_rad = orient[1] 
    
    -- 3. Update Graph (In Degrees for visibility)
    local alpha_deg = alpha_rad * 57.2958
    sim.setGraphStreamValue(graphHandle, tiltStream, alpha_deg)
    
    -- 4. CRITICAL: Send ALPHA to Python Brain
    simROS2.publish(tiltPub, {data = alpha_rad})
    
    -- Debug Print
    print(string.format("Tilt (Alpha): %.2f deg", alpha_deg))
end