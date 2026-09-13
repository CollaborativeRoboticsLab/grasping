from grasping_control.scene_manager_core import SceneManagerError
from grasping_control.scene_manager_node import should_retry_default_scene_activation


def test_should_retry_default_scene_activation_for_moveit_startup_races():
	assert should_retry_default_scene_activation(
		SceneManagerError(
			'planning_scene_unavailable',
			'ApplyPlanningScene service is not available for scene activation.',
		)
	)
	assert should_retry_default_scene_activation(
		SceneManagerError(
			'allowed_collision_matrix_unavailable',
			'GetPlanningScene service is not available to extend the allowed collision matrix.',
		)
	)


def test_should_not_retry_default_scene_activation_for_static_configuration_errors():
	assert not should_retry_default_scene_activation(
		SceneManagerError(
			'workspace_path_resolution_failed',
			"Could not resolve workspace file 'missing.yaml'.",
		)
	)
	assert not should_retry_default_scene_activation(RuntimeError('unexpected failure'))