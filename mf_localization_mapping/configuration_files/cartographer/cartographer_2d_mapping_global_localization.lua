-- Configuration for estimating a new trajectory's pose against a loaded map.
-- This is a diagnostic/initial-alignment variant of cartographer_2d_mapping.lua.

include "cartographer_2d_mapping.lua"

-- Override the default sensor height filter for the Toranomon Hills B2F data.
TRAJECTORY_BUILDER_2D.max_z = 3.0

-- Search the loaded trajectory's submaps globally from the beginning of the
-- additional recording.  The normal mapping configuration intentionally
-- disables this search.
POSE_GRAPH.global_sampling_ratio = 0.1
POSE_GRAPH.global_constraint_search_after_n_seconds = 0.0

-- Use permissive thresholds while finding the coarse inter-trajectory pose;
-- the resulting constraints are still optimized by the pose graph.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
POSE_GRAPH.constraint_builder.min_score = 0.3
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.35

POSE_GRAPH.optimize_every_n_nodes = 5

return options
