# Loading the workspace for planning

The grasping runtime now distinguishes between workspace calibration files and the currently active runtime scene.

- `workspace_creation_node.py` creates or edits workspace YAML files
- `scene_manager_node.py` resolves and activates one workspace file as the current runtime scene
- `motion_execution_node.py` and `feasibility_service_node.py` consume the active scene rather than owning workspace parsing themselves

To run the system without a robot, use the `use_demo:=true` launch parameter.

This checkout currently includes `motion_execution.launch.py` under `grasping_control/launch`. That launch now starts `scene_manager_node` together with the arm execution and optional feasibility nodes.

Each robot stack (TM, UR, etc.) should provide its own launch file for motion execution. The launch file should include the `motion_execution.launch.py` from this package and have its own motion configuration and workspace configuration files.

The robot-specific launch should provide, at minimum:

- a motion configuration file for `motion_execution_node`
- a default scene reference for `scene_manager_node`, typically `default_scene_package` plus `workspace_file`
- matching scene API names when integrating with a mobile-manipulator scene registry

For mobile manipulation, the expected flow is:

1. mobile-manipulator `scene_registry_node` resolves a scene name from `scenes.yaml`
2. grasping `scene_manager_node` activates the referenced workspace file
3. arm execution and feasibility nodes observe the updated `/active_scene` state

Following are some examples,

- [UR10 Manipulator launch](https://github.com/CollaborativeRoboticsLab/grasping_ur/blob/main/grasping_ur/launch/ur10_soft_two_fingers.launch.py)
- [TM12s Manipulator launch](https://github.com/CollaborativeRoboticsLab/grasping_tm/blob/main/grasping_tm/launch/tm12s_soft_two_fingers.launch.py)
- [LD250 & TM12x Mobile Manipulator launch](https://github.com/CollaborativeRoboticsLab/grasping_omron_moma/blob/main/grasping_omron_moma/launch/ld250_tm12x_soft_two_fingers.launch.py)