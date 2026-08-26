# Scene Manager

## Status

The first runtime version is now implemented.

- `scene_manager_node.py` owns active-scene loading, validation, publication, and MoveIt scene application
- `scene_manager_core.py` owns shared workspace-document loading and normalization helpers
- `motion_execution_node.py` and `feasibility_service_node.py` now bootstrap from `GetActiveScene` and stay synced from `/active_scene`
- launch wiring in `grasping_control/launch/motion_execution.launch.py` now starts `scene_manager_node` and can activate a default scene at startup

## Purpose

The scene-manager is a dedicated node that resolves `package_name + workspace_file`, loads workspace data into MoveIt, and publishes the runtime active-scene state so scene activation can change without restarting the arm stack.

## Runtime Ownership

`scene_manager_node` should be the only runtime component that:

- parses `workspace.yaml`
- converts workspace objects to MoveIt collision objects
- applies allowed object-link collision pairs
- updates the active workspace-area filter source
- publishes active-scene metadata

`motion_execution_node` and `feasibility_service_node` should keep motion planning and feasibility logic, but consume active scene state from the scene manager instead of owning scene parsing as long-term runtime state.

## APIs

Services:

- `GetActiveScene` - returns the current active scene metadata
- `ValidateWorkspaceDocument` - validates a workspace document without applying it

Actions:

- `ActivateScene` - activates a specific workspace scene reference
- `LoadSceneFromContent` - loads a workspace document from content and activates it

Topic:

- `/active_scene` - publishes the current active scene metadata

Default launch parameters used by the current bringup:

- `activate_scene_action_name`: `activate_scene`
- `load_scene_from_content_action_name`: `load_scene_from_content`
- `get_active_scene_service_name`: `get_active_scene`
- `validate_workspace_document_service_name`: `validate_workspace_document`
- `active_scene_topic`: `/active_scene`
- `startup_activate_default_scene`: optional bootstrap path for a known package-relative workspace file

## Interaction With Workspace Creation

The `workspace_creation_node` is the calibration and editing frontend. And,

- keep the interactive CLI capture loop
- reuse the shared scene-management core for document normalization and persistence
- avoid merging operator-interactive calibration behavior into the scene-manager runtime API

## Active Scene Rules

For the current single-robot scope:

- only one scene can be active at a time
- only one activation request should run at a time
- successful activation replaces the previous active scene atomically from the caller point of view
- scene changes should be explicit, not hidden inside task execution
- when the requested scene reference already matches the active scene and `force_reload` is false, the node reuses the current active scene instead of reapplying it

## Current Gaps

- `workspace_creation_node.py` still needs a follow-up pass to consume more of the shared scene-management core
- end-to-end runtime validation is still needed against live MoveIt, registry activation, and full Omron bringup