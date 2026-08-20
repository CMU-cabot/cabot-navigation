-- Configuration for extending a loaded 2D map without an initial pose.
-- The loaded trajectories are expected to be frozen by cartographer_node.

include "cartographer_2d_mapping.lua"

-- Include elevated returns that can stabilize matching in large indoor spaces.
TRAJECTORY_BUILDER_2D.max_z = 3.0

-- Disconnected trajectories use MatchFullSubmap immediately. This matcher
-- searches the complete finished submap and does not use the local search
-- window. Once a global constraint connects the trajectories, the narrower
-- local matcher settings inherited from cartographer_2d_mapping.lua apply.
-- One percent is enough to produce several independent candidates while
-- avoiding hundreds of expensive full-submap searches before the first pose
-- graph optimization connects the trajectories.
POSE_GRAPH.global_sampling_ratio = 0.01

-- A disconnected trajectory still enters MatchFullSubmap immediately because
-- it has no connection timestamp.  Once the first inter-trajectory constraint
-- connects it, keep matching local for the rest of an ordinary bag.  Reopening
-- global search after one minute can create false constraints between distant,
-- visually similar corridors and collapse an otherwise correct extension.
POSE_GRAPH.global_constraint_search_after_n_seconds = 360000.0

-- A loaded map can contain many finished submaps. Keep post-connection local
-- matching sparse so each optimization can keep up with real-time playback.
-- Global matching uses the independent ratio above.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.003
POSE_GRAPH.constraint_builder.min_score = 0.35
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.4
POSE_GRAPH.constraint_builder.log_matches = true

-- The first node of a new trajectory is optimized immediately by the pose
-- graph implementation.  Afterwards, use a wider interval because optimizing
-- a loaded pose graph is expensive and blocks ingestion of high-rate IMU data.
POSE_GRAPH.optimize_every_n_nodes = 90

return options
