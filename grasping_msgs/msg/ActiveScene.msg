# Runtime view of the currently active scene on the grasping side.
#
# scene_handle is the runtime token returned by activation. It is not a MoveIt
# planning pipeline id.

grasping_msgs/SceneReference reference
string scene_handle
string workspace_base_frame
bool workspace_area_enabled
geometry_msgs/Point[] workspace_area_corner_points
string[] object_names
builtin_interfaces/Time loaded_at