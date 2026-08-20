-- Mapping configuration for a broad global search when extending an existing map.
-- This is intended for the added trajectory, whose initial pose is only approximate.
include "cartographer_2d_mapping.lua"

-- Override the default sensor height filter for the Toranomon Hills B2F data.
TRAJECTORY_BUILDER_2D.max_z = 3.0

POSE_GRAPH.constraint_builder.max_constraint_distance = 20.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 20.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = 3.14

POSE_GRAPH.global_sampling_ratio = 0.03
POSE_GRAPH.global_constraint_search_after_n_seconds = 0.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05
POSE_GRAPH.constraint_builder.min_score = 0.3
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.35
POSE_GRAPH.optimize_every_n_nodes = 5

return options
