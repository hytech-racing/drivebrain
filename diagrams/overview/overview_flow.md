flowchart TD
    %% Sensor and simulation
    IMU[IMU measurements] --> COMMS[Comms and message adapters]
    GSS[Ground speed measurements] --> COMMS
    LIDAR[LiDAR point clouds] --> COMMS
    SIM[Simulation inputs] --> COMMS

    %% State estimation
    COMMS -->|Timestamped IMU and GSS events| EST_RUNNER[Estimator runner]
    EST_RUNNER --> EST[State estimator]

    EST -->|Latest state snapshot| AUTONOMY[Autonomy and control]
    EST -->|T_odom_from_base history| TF[Shared TransformBuffer]
    EST -->|State and diagnostics| FOX[Foxglove and logging]

    %% LiDAR perception
    COMMS -->|Timestamped point clouds| PERCEPTION_RUNNER[Perception and frontend runner]
    PERCEPTION_RUNNER --> LIDAR_PROCESSOR[LiDAR processor]

    TF -->|Vehicle poses during scan| LIDAR_PROCESSOR

    LIDAR_PROCESSOR -->|Deskewed and filtered clouds| FOX
    LIDAR_PROCESSOR -->|Cone candidates| FRONTEND[SLAM frontend]
    LIDAR_PROCESSOR -->|Candidate and rejection diagnostics| FOX

    %% SLAM frontend
    TF -->|T_odom_from_base at observation time| FRONTEND
    MAP_STATE[Latest optimized map snapshot] --> FRONTEND

    FRONTEND -->|Associated landmark frame| BACKEND_QUEUE[SLAM backend input queue]
    FRONTEND -->|Association and tracking diagnostics| FOX

    %% SLAM backend
    BACKEND_QUEUE --> BACKEND_RUNNER[SLAM backend runner]
    BACKEND_RUNNER --> GRAPH_SLAM[Incremental GraphSLAM]

    GRAPH_SLAM -->|Optimized poses and landmarks| MAP_STATE
    GRAPH_SLAM -->|T_map_from_odom| TF
    GRAPH_SLAM -->|SLAM diagnostics and visualization| FOX

    %% Planning and control
    FRONTEND -.->|Future persistent planner landmark snapshot| PLANNER[Centerline planner]
    TF -.->|T_map_from_base| PLANNER
    PLANNER -.->|Planned path| AUTONOMY
    PLANNER -.->|Path and debug visualization| FOX

    AUTONOMY -->|Steering and torque commands| ACTUATION[Vehicle actuation or simulation]
