import yaml

from grasping_control.scene_manager_core import SceneReference, load_workspace_config_for_editing, scene_handle_from_reference


def test_load_workspace_config_for_editing_normalizes_missing_sections(tmp_path):
    workspace_file = tmp_path / 'workspace.yaml'
    workspace_file.write_text(
        yaml.safe_dump(
            {
                'motion_execution_node': {
                    'ros__parameters': {
                        'workspace': {
                            'base_frame': 'world',
                            'tool_frame': 'tool_tip',
                            'ground_plane_z': 0.0,
                        }
                    }
                }
            }
        ),
        encoding='utf-8',
    )

    workspace_config = load_workspace_config_for_editing(workspace_file, 'world', 'tool_tip', 0.0)

    assert workspace_config['base_frame'] == 'world'
    assert workspace_config['tool_frame'] == 'tool_tip'
    assert workspace_config['workspace_area'] is None
    assert workspace_config['objects'] == []


def test_scene_handle_from_reference_prefers_revision_when_present():
    reference = SceneReference(
        scene_name='crlab_table',
        package_name='grasping_omron_moma',
        workspace_file='config/crlab_table.yaml',
        scene_revision='v1',
    )

    assert scene_handle_from_reference(reference) == 'crlab_table:v1'