from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import CollisionObject
import yaml

from grasping_control.common import find_colcon_workspace_root, load_yaml_dict
from grasping_control.motion_utils import allowed_collision_pairs_from_workspace
from grasping_control.workspace_utils import (
	collision_objects_from_workspace,
	default_workspace_config,
	write_workspace_config,
	workspace_config_from_document,
)


class SceneManagerError(RuntimeError):
	"""
	@brief Error with a machine-readable failure reason for scene-management operations.
	"""

	def __init__(self, failure_reason: str, message: str) -> None:
		super().__init__(message)
		self.failure_reason = failure_reason


@dataclass(frozen=True)
class SceneReference:
	"""
	@brief Logical identifier for a workspace scene source.

	This is a Python-side mirror of the ROS scene reference message so the core
	logic can stay independent from ROS message classes.
	"""

	scene_name: str
	package_name: str
	workspace_file: str
	scene_revision: str = ''


@dataclass(frozen=True)
class SceneActivationData:
	"""
	@brief Validated scene content derived from a workspace document.
	"""

	reference: SceneReference
	workspace_path: Optional[Path]
	workspace_config: Dict[str, Any]
	collision_objects: list[CollisionObject]
	allowed_collision_pairs: list[tuple[str, str]]
	workspace_base_frame: str
	workspace_area_enabled: bool
	object_names: list[str]
	scene_handle: str
	loaded_at: str


def resolve_workspace_path(reference: SceneReference) -> Path:
	"""
	@brief Resolve a workspace file from a package share path or workspace-relative path.

	@param reference Scene reference identifying the source package and file.
	@return Absolute workspace YAML path.
	@throws RuntimeError Raised when the file cannot be resolved.
	"""
	workspace_path = Path(reference.workspace_file).expanduser()
	if workspace_path.is_absolute() and workspace_path.exists():
		return workspace_path

	if reference.package_name:
		package_path = Path(get_package_share_directory(reference.package_name)) / reference.workspace_file
		if package_path.exists():
			return package_path

	workspace_root = find_colcon_workspace_root(Path(__file__))
	if workspace_root is not None:
		candidate = workspace_root / reference.workspace_file
		if candidate.exists():
			return candidate

	raise SceneManagerError(
		'workspace_path_resolution_failed',
		f"Could not resolve workspace file '{reference.workspace_file}'"
		+ (f" from package '{reference.package_name}'." if reference.package_name else '.')
	)


def load_workspace_document(reference: SceneReference) -> Dict[str, Any]:
	"""
	@brief Load a workspace YAML document from the resolved scene reference.

	@param reference Scene reference identifying the source document.
	@return Parsed YAML dictionary.
	"""
	return load_yaml_dict(resolve_workspace_path(reference), {})


def load_workspace_config_for_editing(
	workspace_path: Path,
	default_base_frame: str,
	default_tool_frame: str,
	ground_plane_z: float,
) -> Dict[str, Any]:
	"""
	@brief Load and normalize a workspace document for interactive editing flows.

	@param workspace_path Absolute path to the workspace YAML file.
	@param default_base_frame Fallback workspace base frame.
	@param default_tool_frame Fallback workspace tool frame.
	@param ground_plane_z Fallback ground plane height.
	@return Normalized editable workspace configuration.
	"""
	default_config = default_workspace_config(default_base_frame, default_tool_frame, ground_plane_z)
	workspace_config = workspace_config_from_document(
		load_yaml_dict(workspace_path, default_config),
		default_config,
	)
	workspace_config.setdefault('workspace_area', None)
	workspace_config.setdefault('objects', [])
	return workspace_config


def load_workspace_document_from_content(workspace_document: str) -> Dict[str, Any]:
	"""
	@brief Parse a workspace YAML document from raw string content.

	@param workspace_document YAML content supplied by a client.
	@return Parsed YAML dictionary.
	@throws RuntimeError Raised when the parsed root is not a dictionary.
	"""
	loaded = yaml.safe_load(workspace_document) or {}
	if not isinstance(loaded, dict):
		raise SceneManagerError(
			'invalid_workspace_document',
			'Expected a dictionary at the root of the workspace document.',
		)
	return loaded


def validate_workspace_document(
	reference: SceneReference,
	document: Dict[str, Any],
	default_base_frame: str,
	default_tool_frame: str,
	ground_plane_z: float,
) -> SceneActivationData:
	"""
	@brief Convert a raw workspace document into validated runtime scene data.

	@param reference Scene reference to associate with the validated data.
	@param document Parsed workspace YAML dictionary.
	@param default_base_frame Fallback workspace base frame.
	@param default_tool_frame Fallback tool frame for defaults.
	@param ground_plane_z Fallback ground plane height.
	@return Derived scene activation data suitable for planning-scene updates.
	"""
	default_config = default_workspace_config(default_base_frame, default_tool_frame, ground_plane_z)
	workspace_config = workspace_config_from_document(document, default_config)
	if not isinstance(workspace_config, dict):
		raise SceneManagerError('invalid_workspace_document', 'Workspace document did not produce a valid configuration mapping.')
	collision_objects = collision_objects_from_workspace(workspace_config, default_base_frame)
	object_names = [collision_object.id for collision_object in collision_objects]
	return SceneActivationData(
		reference=reference,
		workspace_path=None,
		workspace_config=workspace_config,
		collision_objects=collision_objects,
		allowed_collision_pairs=allowed_collision_pairs_from_workspace(workspace_config, object_names),
		workspace_base_frame=str(workspace_config.get('base_frame', default_base_frame)),
		workspace_area_enabled=isinstance(workspace_config.get('workspace_area'), dict),
		object_names=object_names,
		scene_handle=scene_handle_from_reference(reference),
		loaded_at=iso_timestamp(),
	)


def load_scene_from_reference(
	reference: SceneReference,
	default_base_frame: str,
	default_tool_frame: str,
	ground_plane_z: float,
) -> SceneActivationData:
	"""
	@brief Resolve, load, and validate a workspace scene from a reference.

	@param reference Scene reference identifying the workspace file.
	@param default_base_frame Fallback workspace base frame.
	@param default_tool_frame Fallback tool frame for defaults.
	@param ground_plane_z Fallback ground plane height.
	@return Derived scene activation data.
	"""
	workspace_path = resolve_workspace_path(reference)
	activation = validate_workspace_document(
		reference,
		load_yaml_dict(workspace_path, {}),
		default_base_frame,
		default_tool_frame,
		ground_plane_z,
	)
	return SceneActivationData(
		reference=activation.reference,
		workspace_path=workspace_path,
		workspace_config=activation.workspace_config,
		collision_objects=activation.collision_objects,
		allowed_collision_pairs=activation.allowed_collision_pairs,
		workspace_base_frame=activation.workspace_base_frame,
		workspace_area_enabled=activation.workspace_area_enabled,
		object_names=activation.object_names,
		scene_handle=activation.scene_handle,
		loaded_at=activation.loaded_at,
	)


def persist_workspace_document(
	path: Path,
	workspace_config: Dict[str, Any],
	base_frame: str,
	tool_frame: str,
	ground_plane_z: float,
) -> Dict[str, Any]:
	"""
	@brief Persist a normalized workspace document through the existing helper.

	@param path Destination path.
	@param workspace_config Existing workspace configuration.
	@param base_frame Workspace base frame.
	@param tool_frame Workspace tool frame.
	@param ground_plane_z Workspace ground plane height.
	@return Normalized configuration written to disk.
	"""
	return write_workspace_config(path, workspace_config, base_frame, tool_frame, ground_plane_z)


def default_workspace_save_location(
	workspace_config_path: Path,
	workspace_root: Optional[Path],
) -> tuple[Path, Optional[Path]]:
	"""
	@brief Derive the default save root and overwrite target for workspace editing flows.

	@param workspace_config_path Current workspace document path used for editing.
	@param workspace_root Optional detected colcon workspace root.
	@return Tuple of save root and default overwrite target.
	"""
	default_save_path: Optional[Path] = None
	save_root = workspace_config_path.parent
	if workspace_config_path.name != 'workspace_empty.yaml':
		default_save_path = workspace_config_path
	elif workspace_root is not None:
		save_root = workspace_root
	return save_root, default_save_path


def normalize_workspace_save_path(response: str, save_root: Path) -> Path:
	"""
	@brief Normalize a user-provided workspace save destination.

	@param response Raw user input from the save prompt.
	@param save_root Base directory for relative save paths.
	@return Absolute destination path with a YAML suffix.
	"""
	save_path = Path(response).expanduser()
	if not save_path.is_absolute():
		save_path = (save_root / save_path).resolve()
	if save_path.suffix not in {'.yaml', '.yml'}:
		save_path = save_path.with_suffix('.yaml')
	return save_path


def scene_handle_from_reference(reference: SceneReference) -> str:
	"""
	@brief Build a deterministic runtime scene handle from a scene reference.

	@param reference Scene reference identifying the source scene.
	@return Stable scene handle string.
	"""
	base_name = reference.scene_name or Path(reference.workspace_file).stem or 'scene'
	if reference.scene_revision:
		return f'{base_name}:{reference.scene_revision}'
	return base_name


def iso_timestamp() -> str:
	"""
	@brief Return the current UTC time as an ISO-8601 string.

	@return Timestamp string suitable for active-scene metadata.
	"""
	return datetime.now(timezone.utc).isoformat()


def scene_references_match(left: SceneReference, right: SceneReference) -> bool:
	"""
	@brief Return whether two scene references identify the same logical scene source.

	@param left First scene reference.
	@param right Second scene reference.
	@return True when scene name, package, path, and revision are equal after trimming.
	"""
	return (
		left.scene_name.strip() == right.scene_name.strip()
		and left.package_name.strip() == right.package_name.strip()
		and left.workspace_file.strip() == right.workspace_file.strip()
		and left.scene_revision.strip() == right.scene_revision.strip()
	)