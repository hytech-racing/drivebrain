flowchart LR
    CONES[Cone candidates] --> FE[SLAM frontend]
    ODOM[Local vehicle pose] --> FE
    MAP[Latest optimized map snapshot] --> FE

    FE -->|Associated landmark observations| BE[SLAM backend]

    BE -->|Optimized map snapshot| MAP
    BE -->|Map-to-odom correction| TF[Transform buffer]
    FE -.->|Persistent planner landmarks| PLAN[Planner]
