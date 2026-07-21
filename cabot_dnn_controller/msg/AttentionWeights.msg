# Inference time and the coordinate frame used for the people input.
std_msgs/Header header

# Row-major weights with shape [num_heads, num_queries, num_keys].
uint32 num_heads
uint32 num_queries
uint32 num_keys
float32[] weights
