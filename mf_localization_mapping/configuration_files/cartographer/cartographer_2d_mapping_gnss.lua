-- Copyright (c) 2021  IBM Corporation
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in all
-- copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
-- SOFTWARE.

include "cartographer_2d_mapping.lua"

-- Handle NavSatFix topic
options.use_nav_sat = true

-- Customized settings to convert geodetic coordinate to ENU (East-North-Up) coordinate by spherical mercator projection
options.nav_sat_use_enu_local_frame = true  -- default: false
options.nav_sat_use_spherical_mercator = true  -- default: false

-- Use predifined ENU frame at specific point (latitude and logitude) if necessary. If not, the first NavSatFix message is used to initialize ECEF to local frame conversion
-- options.nav_sat_use_predefined_enu_frame = true
-- options.nav_sat_predefined_enu_frame_latitude = (float)
-- options.nav_sat_predefined_enu_frame_longitude = (float)

-- Tune this value if the submap data size becomes too large
-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- default: 0.05

-- Disable collate_fixed_frame to allow non continous (intermittent) fixed frame inputs
TRAJECTORY_BUILDER.collate_fixed_frame = false -- default: true

-- Settings for optimization problem using fixed frame constraint
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e4 -- default: 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 0.0 -- GPS provides no orientation
POSE_GRAPH.optimization_problem.fixed_frame_pose_use_tolerant_loss = true -- default: false

-- Customized settings to align cartographer coordinate to ENU coordinate.
POSE_GRAPH.optimization_problem.set_constant_fixed_frame_origin = true  -- default: false
POSE_GRAPH.optimization_problem.zero_initialize_fixed_frame_origin = true  -- default: false
POSE_GRAPH.optimization_problem.set_constant_first_submap_translation = true  -- default: true
POSE_GRAPH.optimization_problem.set_constant_first_submap_rotation = false -- default: true

return options
