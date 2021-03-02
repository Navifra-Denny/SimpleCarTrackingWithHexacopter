# state
bool global_to_local
bool is_detected
bool is_reached
bool is_hover
bool is_global
string detection_tool

# waypoint
geometry_msgs/Point local_waypoint_position
geographic_msgs/GeoPoint global_waypoint_position

# position
geometry_msgs/Point curr_local_position
geographic_msgs/GeoPoint curr_global_position