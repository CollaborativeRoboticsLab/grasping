# Mobile-manipulator-owned scene registry entry.
#
# This message is returned by ListAvailableScenes and identifies a scene that
# can later be activated through ActivateSceneByName.

grasping_msgs/SceneReference reference
string station_frame
string[] semantic_tags
string[] allowed_modes
string selector_profile
string description