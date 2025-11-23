##
## Editor-only helper script for VerletRope addon.
## This script exists to isolate EditorInterface usage from the main verlet_rope.gd script.
##
## Why this is necessary:
## EditorInterface is not available in exported builds and causes parse errors when referenced
## directly in scripts that are included in exports. By isolating EditorInterface calls in this
## separate script and loading it conditionally at runtime (only in editor mode), we avoid
## parse errors in exported builds while maintaining editor functionality.
##
## Related issue: https://github.com/godotengine/godot/issues/91713
##
static func get_camera() -> Camera3D:
	return EditorInterface.get_editor_viewport_3d(0).get_camera_3d()
