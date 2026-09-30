# DRILLPULSE — Navigation & Autonomous Exploration Architecture

## 1. Mode Separation (Explored vs Unexplored)
- Explored Mode: AMCL active on pre-built survey map (maps/rover_map.yaml). SLAM Toolbox disabled.
- Unexplored Mode: SLAM Toolbox active for online graph mapping. AMCL disabled.

## 2. Frontier Exploration & BFS Clustering
- Ingests 2D costmap (/map) at 0.05m resolution.
- Identifies free-to-unknown boundary cells.
- Clusters adjacent boundary cells via 8-connected BFS.
- Scores candidates: Score = w_gain * Gain - w_dist * Dist - w_heading * dTheta.
- Dispatches top centroid via Nav2 NavigateToPose action client.
