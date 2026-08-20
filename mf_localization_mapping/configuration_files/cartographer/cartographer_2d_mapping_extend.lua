-- Configuration for extending a loaded 2D map, with or without an initial pose.
-- The loaded trajectories are expected to be frozen by cartographer_node.

include "cartographer_2d_mapping.lua"

-- Include elevated returns that can stabilize matching in large indoor spaces.
TRAJECTORY_BUILDER_2D.max_z = 3.0

-- When an initial trajectory pose is supplied, it marks the added trajectory
-- as connected to the reference trajectory. For the first minute,
-- Cartographer uses the seeded local matcher with this broad window.
POSE_GRAPH.constraint_builder.max_constraint_distance = 20.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 20.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.pi

-- Without an initial pose, the trajectories are disconnected and Cartographer
-- uses full-submap global matching immediately. With a seed, the same matcher
-- becomes a fallback if no new inter-trajectory constraint is found for one
-- minute. MatchFullSubmap does not use the 20 m local search window above.
POSE_GRAPH.global_sampling_ratio = 0.03
POSE_GRAPH.global_constraint_search_after_n_seconds = 60.0

-- Keep the broad matcher sparse enough that playback and final optimization
-- do not build an excessive work queue. This is still ten times the default
-- global sampling ratio and matches the proven RSS initialization strategy.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.constraint_builder.min_score = 0.35
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.4
POSE_GRAPH.constraint_builder.log_matches = true

-- Optimizing a loaded pose graph is considerably more expensive than
-- optimizing a map built from scratch.  A short interval stalls pose-graph
-- ingestion and lets high-rate IMU work items accumulate during playback.
POSE_GRAPH.optimize_every_n_nodes = 90

return options
