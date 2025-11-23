# MIT License
#
# Copyright (c) 2021 Shashank C; Copyright (c) 2023 Zae Chao(zaevi); No Copyright (-c) Tshmofen / Timofey Ivanov
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

@tool
extends MeshInstance3D
class_name VerletRope

# RopeCollisionType.gd
enum RopeCollisionType {
	STATIC_ONLY,
	DYNAMIC_ONLY,
	ALL
}

# RopeCollisionBehavior.gd
enum RopeCollisionBehavior {
	NONE,
	STICKY_STRETCH,
	SLIDE_STRETCH
}

class RopeRaycastCollisionData:
	var position: Vector3
	var normal: Vector3
	
	func _init(position: Vector3, normal: Vector3) -> void:
		print("pos")
		self.position = position
		self.normal = normal

class RopeCollisionInfo:
	var is_static_collision: bool
	var dynamic_collisions: Array[Vector3]
	
	func _init(is_static_collision: bool, dynamic_collisions: Array[Vector3]) -> void:
		self.is_static_collision = is_static_collision
		self.dynamic_collisions = dynamic_collisions

# --- Signals ---
signal simulation_step_event_handler(delta: float)

# --- Constants ---
const DEFAULT_MATERIAL_PATH := "res://addons/verlet_rope_4_gd/materials/rope_default.material"
const NO_NOTIFIER_WARNING := "Consider checking 'UseVisibleOnScreenNotifier' to disable rope visuals when it's not on screen for increased performance."
const CREATION_STAMP_META := "creation_stamp"
const COLLISION_CHECK_LENGTH := 0.001
const DYNAMIC_COLLISION_CHECK_LENGTH := 0.08

const COS_5_DEG := cos(deg_to_rad(5.0))
const COS_15_DEG := cos(deg_to_rad(15.0))
const COS_30_DEG := cos(deg_to_rad(30.0))

# --- Private Variables ---
var _time: float = 0.0
var _camera: Camera3D
var _simulation_delta: float = 0.0
var _particle_data: RopeParticleData
var _visible_notifier: VisibleOnScreenNotifier3D

var _ray_cast: RayCast3D
var _mesh: ImmediateMesh
var _collision_check_box: BoxShape3D
var _space_state: PhysicsDirectSpaceState3D
var _collision_shape_parameters: PhysicsShapeQueryParameters3D

var _simulation_particles: int = 10
## Cached editor script to avoid repeated load() calls in editor mode (see editor.gd for details)
var _editor_script: GDScript

@export_group("Basics")

## Total length of the rope in meters
@export var rope_length: float = 5.0

## Visual thickness of the rope mesh
@export var rope_width: float = 0.07

## Number of simulation points along the rope (higher values = smoother but more expensive)
@export_range(3, 300) var simulation_particles: int = 10:
	set(value):
		simulation_particles = value
		if _particle_data == null:
			return
		_particle_data.resize(simulation_particles)
		create_rope()

## Automatically disable rope rendering when off-screen for performance optimization
@export var use_visible_on_screen_notifier: bool = true:
	set(value):
		use_visible_on_screen_notifier = value
		update_configuration_warnings()

@export_group("Attachment")

## NodePath to a Node3D that will follow the rope's end position
@export var payload_object: NodePath
var _editor_payload_object: Node3D = null

## If true, the attached object will rotate to match the rope's end orientation
@export var rotate_payload_object: bool = false

# Node that will follow the end of the rope (programmatic attachment)
var _payload_object: Node3D = null

var _attach_start := true

## If true, the rope's start point is fixed to this node's position
@export var attach_start: bool = true:
	set(value):
		attach_start = value
		_attach_start = value
		if _particle_data != null:
			_particle_data.particles[0].is_attached = value

var _attach_end: Node3D = null

## Node3D to attach the rope's end point to (will follow that node's position)
@export var attach_end: Node3D:
	set(value):
		attach_end = value
		_attach_end = value
		if _particle_data != null:
			_particle_data.particles[_particle_data.particles.size() - 1].is_attached = value != null


@export_group("Simulation")

## Physics update rate for rope simulation in Hz (higher = more stable but more expensive)
@export_range(30, 265) var simulation_rate: int = 60

## Number of constraint solver iterations per simulation step (higher = more rigid)
@export var iterations: int = 2

## Number of iterations to run during rope initialization to settle the rope
@export var preprocess_iterations: int = 5

## Delta time used for each preprocess iteration step
@export var preprocess_delta: float = 0.016

## How rigid the rope is (0.0 = very elastic, 1.5 = very stiff)
@export_range(0.0, 1.5) var stiffness: float = 0.9

## If true, rope initializes from start point; if false, rope hangs down immediately
@export var start_simulation_from_start_point: bool = true

## Enable or disable physics simulation
@export var simulate: bool = true

## Enable or disable rope mesh rendering
@export var draw: bool = true

## Automatically start simulation and drawing when entering the scene
@export var start_draw_simulation_on_start: bool = true

## Distance from camera where rope mesh subdivision detail starts to reduce
@export var subdivision_lod_distance: float = 15.0

@export_group("Gravity")

## Enable or disable gravity force on the rope
@export var apply_gravity: bool = true

## Gravity vector direction and magnitude
@export var gravity: Vector3 = Vector3.DOWN * 9.8

## Multiplier for gravity strength
@export var gravity_scale: float = 1.0

@export_group("Wind")

## Enable or disable wind force on the rope
@export var apply_wind: bool = false

## Noise texture used to create turbulent wind patterns
@export var wind_noise: FastNoiseLite = null

## Base wind direction and strength
@export var wind: Vector3 = Vector3(1.0, 0.0, 0.0)

## Multiplier for wind force intensity
@export var wind_scale: float = 20.0

@export_group("Damping")

## Enable or disable velocity damping (air resistance)
@export var apply_damping: bool = true

## Strength of damping force (higher = rope slows down faster)
@export var damping_factor: float = 100.0

@export_group("Collision")

## Type of objects the rope can collide with (StaticOnly, DynamicOnly, or All)
@export_enum("StaticOnly", "DynamicOnly", "All")
var rope_collision_type: int = 0

## How rope behaves during collisions (None, StickyStretch stops when stretched, SlideStretch slides along surfaces)
@export_enum("None", "StickyStretch", "SlideStretch")
var rope_collision_behavior: int = 0

## Maximum rope stretch multiplier before collision behavior activates
@export_range(1.0, 20.0) var max_rope_stretch: float = 1.1

## Stretch threshold where SlideStretch mode ignores collisions to allow sliding
@export_range(1.0, 20.0) var slide_ignore_collision_stretch: float = 1.5

## Maximum number of dynamic collision objects to check per frame
@export_range(1, 256) var max_dynamic_collisions: int = 32

var _static_collision_mask: int = 1

## Physics layers for static collision detection
@export_flags_3d_physics var static_collision_mask: int = 1:
	set(value):
		static_collision_mask = value
		_static_collision_mask = value
		if _ray_cast:
			_ray_cast.collision_mask = value
		if _collision_shape_parameters:
			_collision_shape_parameters.collision_mask = value

## Physics layers for dynamic collision detection
@export_flags_3d_physics var dynamic_collision_mask: int = 0

var _hit_from_inside: bool = true

## Allow rope to detect collisions from inside collision shapes
@export var hit_from_inside: bool = true:
	set(value):
		hit_from_inside = value
		_hit_from_inside = value
		if _ray_cast:
			_ray_cast.hit_from_inside = value

var _hit_back_faces: bool = true

## Allow rope to collide with back faces of mesh colliders
@export var hit_back_faces: bool = true:
	set(value):
		hit_back_faces = value
		_hit_back_faces = value
		if _ray_cast:
			_ray_cast.hit_back_faces = value

@export_group("Debug")

## Draw debug lines showing particle tangent, normal, and binormal vectors
@export var draw_debug_particles: bool = false

# UTIL
func get_simulation_particles(index: int) -> Array[Vector3]:
	var segment_length = get_average_segment_length()

	var p0: Vector3
	if index == 0:
		p0 = _particle_data.particles[index].position_current - (_particle_data.particles[index].tangent * segment_length)
	else:
		p0 = _particle_data.particles[index - 1].position_current

	var p1: Vector3 = _particle_data.particles[index].position_current
	var p2: Vector3 = _particle_data.particles[index + 1].position_current

	var p3: Vector3
	if index == simulation_particles - 2:
		p3 = _particle_data.particles[index + 1].position_current + (_particle_data.particles[index + 1].tangent * segment_length)
	else:
		p3 = _particle_data.particles[index + 2].position_current

	return [p0, p1, p2, p3]


func get_average_segment_length() -> float:
	return rope_length / float(simulation_particles - 1)

func reset_rope_rotation() -> void:
	# Rope doesn't draw correctly when rotated, so we reset rotation
	global_transform = Transform3D(Basis.IDENTITY, global_position)

func get_current_rope_length() -> float:
	var length := 0.0
	for i in range(simulation_particles - 1):
		length += (_particle_data.particles[i + 1].position_current - _particle_data.particles[i].position_current).length()
	return length

func collide_raycast(from: Vector3, direction: Vector3, collision_mask: int) -> RopeRaycastCollisionData:
	# Assumes _ray_cast is a RayCast3D node already added to the scene and referenced
	_ray_cast.collision_mask = collision_mask
	_ray_cast.global_position = from
	_ray_cast.target_position = direction
	_ray_cast.force_raycast_update()

	if not _ray_cast.is_colliding():
		return null

	return RopeRaycastCollisionData.new(
		_ray_cast.get_collision_point(),
		_ray_cast.get_collision_normal()
	)

# DRAW
func catmull_interpolate(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3, tension: float, t: float) -> Dictionary:
	var t_sqr = t * t
	var t_cube = t_sqr * t

	var m1 = (1.0 - tension) / 2.0 * (p2 - p0)
	var m2 = (1.0 - tension) / 2.0 * (p3 - p1)

	var a = (2.0 * (p1 - p2)) + m1 + m2
	var b = (-3.0 * (p1 - p2)) - (2.0 * m1) - m2

	var point = (a * t_cube) + (b * t_sqr) + (m1 * t) + p1
	var tangent = ((3.0 * a * t_sqr) + (2.0 * b * t) + m1).normalized()
	
	return {"point": point, "tangent": tangent}

func draw_quad(vertices: Array[Vector3], normal: Vector3, uvx0: float, uvx1: float) -> void:
	_mesh.surface_set_normal(normal)
	
	_mesh.surface_set_uv(Vector2(uvx0, 0.0))
	_mesh.surface_add_vertex(vertices[0])

	_mesh.surface_set_uv(Vector2(uvx1, 0.0))
	_mesh.surface_add_vertex(vertices[1])

	_mesh.surface_set_uv(Vector2(uvx1, 1.0))
	_mesh.surface_add_vertex(vertices[2])

	_mesh.surface_set_uv(Vector2(uvx0, 0.0))
	_mesh.surface_add_vertex(vertices[0])

	_mesh.surface_set_uv(Vector2(uvx1, 1.0))
	_mesh.surface_add_vertex(vertices[2])

	_mesh.surface_set_uv(Vector2(uvx0, 1.0))
	_mesh.surface_add_vertex(vertices[3])

func get_draw_subdivision_step(camera_position: Vector3, particle_index: int) -> float:
	var cam_dist_particle = camera_position - _particle_data.particles[particle_index].position_current

	if cam_dist_particle.length_squared() > subdivision_lod_distance * subdivision_lod_distance:
		return 1.0

	# Safety check to ensure we're not accessing out of bounds
	if particle_index + 1 >= _particle_data.particles.size():
		return 1.0
		
	var tangent_dots = _particle_data.particles[particle_index].tangent.dot(_particle_data.particles[particle_index + 1].tangent)

	if tangent_dots >= COS_5_DEG:
		return 1.0
	elif tangent_dots >= COS_15_DEG:
		return 0.5
	elif tangent_dots >= COS_30_DEG:
		return 0.33333
	else:
		return 0.25

func calculate_rope_camera_orientation() -> void:
	var camera_position := Vector3.ZERO
	if _camera:
		camera_position = _camera.global_position

	var start := _particle_data.particles[0]
	start.tangent = (_particle_data.particles[1].position_current - start.position_current).normalized()
	start.normal = (start.position_current - camera_position).normalized()
	start.binormal = start.normal.cross(start.tangent).normalized()
	_particle_data.particles[0] = start

	var end_index := simulation_particles - 1
	var end := _particle_data.particles[end_index]
	end.tangent = (end.position_current - _particle_data.particles[end_index - 1].position_current).normalized()
	end.normal = (end.position_current - camera_position).normalized()
	end.binormal = end.normal.cross(end.tangent).normalized()
	_particle_data.particles[end_index] = end

	for i in range(1, simulation_particles - 1):
		var particle := _particle_data.particles[i]
		particle.tangent = (_particle_data.particles[i + 1].position_current - _particle_data.particles[i - 1].position_current).normalized()
		particle.normal = (particle.position_current - camera_position).normalized()
		particle.binormal = particle.normal.cross(particle.tangent).normalized()
		_particle_data.particles[i] = particle

# CONSTRAINTS
func stiff_rope() -> void:
	for iteration in iterations:
		for i in range(simulation_particles - 1):
			var p0 = _particle_data.particles[i]
			var p1 = _particle_data.particles[i + 1]

			var segment = p1.position_current - p0.position_current
			var stretch = segment.length() - get_average_segment_length()
			var direction = segment.normalized()

			if p0.is_attached:
				p1.position_current -= direction * stretch * stiffness
			elif p1.is_attached:
				p0.position_current += direction * stretch * stiffness
			else:
				var half_stretch = direction * stretch * 0.5 * stiffness
				p0.position_current += half_stretch
				p1.position_current -= half_stretch

			_particle_data.particles[i] = p0
			_particle_data.particles[i + 1] = p1

func get_rope_collisions() -> RopeCollisionInfo:
	var visuals = get_aabb()

	if visuals.size == Vector3.ZERO:
		return RopeCollisionInfo.new(false, [])

	_collision_check_box.size = visuals.size
	_collision_shape_parameters.transform = Transform3D(
		_collision_shape_parameters.transform.basis,
		global_position + visuals.position + (visuals.size / 2.0)
	)

	var is_static_collision := false
	if RopeCollisionType in [RopeCollisionType.ALL, RopeCollisionType.STATIC_ONLY]:
		_collision_shape_parameters.collision_mask = static_collision_mask
		is_static_collision = _space_state.collide_shape(_collision_shape_parameters, 1).size() > 0

	var dynamic_collisions: Array[Vector3] = []

	if RopeCollisionType in [RopeCollisionType.ALL, RopeCollisionType.DYNAMIC_ONLY]:
		_collision_shape_parameters.collision_mask = dynamic_collision_mask
		var results = _space_state.intersect_shape(_collision_shape_parameters, max_dynamic_collisions)
		for hit in results:
			if hit.has("collider") and hit["collider"] is Node3D:
				dynamic_collisions.append(hit["collider"].global_position)

	return RopeCollisionInfo.new(
		is_static_collision,
		dynamic_collisions
	)

func collide_rope(dynamic_collisions: Array) -> void:
	var general_collision_mask := static_collision_mask
	match RopeCollisionType:
		RopeCollisionType.ALL:
			general_collision_mask = static_collision_mask | dynamic_collision_mask
		RopeCollisionType.DYNAMIC_ONLY:
			general_collision_mask = dynamic_collision_mask
		RopeCollisionType.STATIC_ONLY:
			general_collision_mask = static_collision_mask

	var segment_slide_ignore_length = get_average_segment_length() * slide_ignore_collision_stretch
	var is_rope_stretched = get_current_rope_length() > rope_length * max_rope_stretch

	for i in range(1, simulation_particles):
		var current_point = _particle_data.particles[i]

		if is_rope_stretched:
			if rope_collision_behavior == RopeCollisionBehavior.STICKY_STRETCH:
				continue

			var previous_point = _particle_data.particles[i - 1]
			var segment_length = (previous_point.position_current - current_point.position_current).length()
			if segment_length > segment_slide_ignore_length:
				continue

		for dynamic_collision in dynamic_collisions:
			var to_dynamic = (dynamic_collision - current_point.position_current).normalized() * DYNAMIC_COLLISION_CHECK_LENGTH
			var collision_data = collide_raycast(current_point.position_current, to_dynamic, dynamic_collision_mask)
			if collision_data != null:
				current_point.position_current = collision_data.position + collision_data.normal * DYNAMIC_COLLISION_CHECK_LENGTH

		var particle_move = current_point.position_current - current_point.position_previous
		if particle_move == Vector3.ZERO:
			continue

		var general_to = particle_move + particle_move.normalized() * COLLISION_CHECK_LENGTH
		var general_data = collide_raycast(current_point.position_previous, general_to, general_collision_mask)
		if general_data == null:
			continue

		current_point.position_current = general_data.position + general_data.normal * COLLISION_CHECK_LENGTH

		if is_rope_stretched:
			current_point.position_current += particle_move.slide(general_data.normal)

		_particle_data.particles[i] = current_point


func _get_camera() -> Camera3D:
	var cam := get_viewport().get_camera_3d()
	
	if Engine.is_editor_hint():
		if not _editor_script:
			_editor_script = load("res://addons/verlet_rope_4_gd/editor.gd")
		return _editor_script.get_camera()
	
	return cam

func draw_curve() -> void:
	# Ensure mesh exists
	if _mesh == null:
		print("Error: No mesh available for drawing curve")
		return
		
	_mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES)

	# Ensure camera exists for proper rendering
	if _camera == null:
		_camera = _get_camera()
		if _camera == null:
			print("Warning: No camera found for rope rendering")
			return

	var camera_position: Vector3 = _camera.global_position

	for i in simulation_particles - 1:
		var particles := get_simulation_particles(i)
		var p0 = particles[0]
		var p1 = particles[1]
		var p2 = particles[2]
		var p3 = particles[3]
		
		var step = get_draw_subdivision_step(camera_position, i)
		var t := 0.0

		while t <= 1.0:
			var current_result = catmull_interpolate(p0, p1, p2, p3, 0.0, t)
			var current_position = current_result["point"]
			var current_tangent = current_result["tangent"]

			var next_result = catmull_interpolate(p0, p1, p2, p3, 0.0, min(t + step, 1.0))
			var next_position = next_result["point"]
			var next_tangent = next_result["tangent"]

			var current_normal = (current_position - camera_position).normalized()
			var current_binormal = current_normal.cross(current_tangent).normalized()
			current_position -= global_position

			var next_normal = (next_position - camera_position).normalized()
			var next_binormal = next_normal.cross(next_tangent).normalized()
			next_position -= global_position

			var vs: Array[Vector3] = [
				current_position - (current_binormal * rope_width),
				next_position - (next_binormal * rope_width),
				next_position + (next_binormal * rope_width),
				current_position + (current_binormal * rope_width)
			]

			draw_quad(vs, -current_binormal, t, t + step)
			t += step

	_mesh.surface_end()

func verlet_process(delta: float) -> void:
	for i in range(simulation_particles):
		var p = _particle_data.particles[i]

		if p.is_attached:
			continue

		var position_current_copy = p.position_current
		p.position_current = (2.0 * p.position_current) - p.position_previous + (delta * delta * p.acceleration)
		p.position_previous = position_current_copy

		_particle_data.particles[i] = p # write back the modified particle if using array of dictionaries or structs

func apply_forces() -> void:
	for i in simulation_particles:
		var p := _particle_data.particles[i]
		var total_acceleration: Vector3 = Vector3.ZERO

		if apply_gravity:
			total_acceleration += gravity * gravity_scale

		if apply_wind and wind_noise != null:
			var timed_position := p.position_current + Vector3.ONE * _time
			var wind_force := wind_noise.get_noise_3d(timed_position.x, timed_position.y, timed_position.z)
			total_acceleration += wind_scale * wind * wind_force

		if apply_damping:
			var velocity := p.position_current - p.position_previous
			var drag := -damping_factor * velocity.length() * velocity
			total_acceleration += drag

		p.acceleration = total_acceleration

func apply_constraints() -> void:
	stiff_rope()

	var is_layers_available := (dynamic_collision_mask != 0 or static_collision_mask != 0) and (
		rope_collision_type == RopeCollisionType.ALL
		or (rope_collision_type == RopeCollisionType.STATIC_ONLY and static_collision_mask != 0)
		or (rope_collision_type == RopeCollisionType.DYNAMIC_ONLY and dynamic_collision_mask != 0)
	)

	if rope_collision_behavior == RopeCollisionBehavior.NONE or not is_layers_available:
		return

	var collision_result := get_rope_collisions()
	var is_static_collision = collision_result.is_static_collision
	var dynamic_collisions = collision_result.dynamic_collisions

	if not is_static_collision and dynamic_collisions.is_empty():
		return

	collide_rope(dynamic_collisions)

func get_configuration_warnings() -> PackedStringArray:
	if use_visible_on_screen_notifier:
		return PackedStringArray()
	else:
		return PackedStringArray([NO_NOTIFIER_WARNING])

func _ready() -> void:
	if not Engine.is_editor_hint() and start_draw_simulation_on_start:
		draw = true
		simulate = true

	# Process object attached via editor
	_editor_payload_object = null
	if not payload_object.is_empty():
		var node = get_node_or_null(payload_object)
		if node is Node3D:
			_editor_payload_object = node
		else:
			push_warning("Attached object path is not a Node3D")

	# Ensure ImmediateMesh exists and is unique to this instance
	#_mesh = Mesh as ImmediateMesh
	if _mesh == null or (_mesh.get_meta("creation_stamp", 0) as int) != get_instance_id():
		_mesh = ImmediateMesh.new()
		_mesh.set_meta("creation_stamp", get_instance_id())
		_mesh.resource_local_to_scene = true
		mesh = _mesh

	# Add raycast for collision
	_ray_cast = RayCast3D.new()
	_ray_cast.collision_mask = static_collision_mask
	_ray_cast.hit_from_inside = _hit_from_inside
	_ray_cast.hit_back_faces = _hit_back_faces
	_ray_cast.enabled = false
	add_child(_ray_cast)

	# Add visibility notifier if enabled
	if use_visible_on_screen_notifier:
		_visible_notifier = VisibleOnScreenNotifier3D.new()
		_visible_notifier.connect("screen_entered", Callable(self, "_on_screen_entered"))
		_visible_notifier.connect("screen_exited", Callable(self, "_on_screen_exited"))
		add_child(_visible_notifier)

	# Camera and space state references
	_camera = _get_camera()
	if _camera == null:
		push_warning("No camera found during rope initialization, rope may not render correctly")
	
	_space_state = get_world_3d().direct_space_state

	# Collision shape setup
	var visuals := get_aabb()
	_collision_check_box = BoxShape3D.new()
	_collision_check_box.size = visuals.size

	_collision_shape_parameters = PhysicsShapeQueryParameters3D.new()
	_collision_shape_parameters.shape_rid = _collision_check_box.get_rid()
	_collision_shape_parameters.collision_mask = static_collision_mask
	_collision_shape_parameters.margin = 0.1
	_collision_shape_parameters.transform = Transform3D.IDENTITY.translated(global_position + visuals.position + visuals.size * 0.5)

	# Load default material if not set
	if material_override == null:
		var mat_path = DEFAULT_MATERIAL_PATH
		if ResourceLoader.exists(mat_path):
			material_override = load(mat_path)
		else:
			# Fallback to a basic material if the default isn't found
			print("Rope material not found at " + mat_path + ", creating default material")
			var mat = StandardMaterial3D.new()
			mat.albedo_color = Color(0.5, 0.5, 0.5) # Gray color
			mat.roughness = 0.8
			material_override = mat

	# Generate rope particles
	create_rope()

func _physics_process(delta: float) -> void:
	if Engine.is_editor_hint():
		if _particle_data == null:
			print("Creating rope in editor")
			create_rope()
		
		# Update editor attachment (if any)
		if _particle_data != null and not payload_object.is_empty():
			update_attached_object_editor()
			

	_time += delta
	_simulation_delta += delta

	var simulation_step := 1.0 / float(simulation_rate)
	if _simulation_delta < simulation_step:
		return

	if _attach_end != null:
		_particle_data.particles[simulation_particles - 1].position_current = _attach_end.global_position

	if attach_start:
		_particle_data.particles[0].position_current = global_position

	if simulate:
		apply_forces()
		verlet_process(_simulation_delta)
		apply_constraints()
		
	# Update position of any attached objects
	update_attached_objects()

	if draw:
		_camera = _get_camera()
		calculate_rope_camera_orientation()
		_mesh.clear_surfaces()
		reset_rope_rotation()
		draw_rope_debug_particles()
		draw_curve()

		if _visible_notifier != null:
			_visible_notifier.aabb = get_aabb()

	emit_signal("simulation_step_event_handler", _simulation_delta)
	_simulation_delta = 0.0

func create_rope() -> void:
	var end_location := global_position + Vector3.DOWN * rope_length

	print("Creating rope with ", simulation_particles, " particles")

	if _attach_end != null:
		end_location = _attach_end.global_position
	elif start_simulation_from_start_point:
		end_location = global_position

	var acceleration := gravity * gravity_scale
	var segment := get_average_segment_length()

	_particle_data = RopeParticleData.generate_particle_data(
		end_location,
		global_position,
		acceleration,
		simulation_particles,
		segment
	)

	var start = _particle_data.particles[0]
	var end = _particle_data.particles[simulation_particles - 1]

	start.is_attached = attach_start
	end.is_attached = _attach_end != null
	end.position_previous = end_location
	end.position_current = end_location

	for i in range(preprocess_iterations):
		verlet_process(preprocess_delta)
		apply_constraints()

	calculate_rope_camera_orientation()

func destroy_rope() -> void:
	_particle_data.resize(0)
	simulation_particles = 0

func draw_rope_debug_particles() -> void:
	const DEBUG_PARTICLE_LENGTH := 0.3
	
	if not draw_debug_particles:
		return
		
	_mesh.surface_begin(Mesh.PRIMITIVE_LINES)

	for i in simulation_particles:
		var particle = _particle_data.particles[i]
		var local_position = particle.position_current - global_position

		# Tangent line
		_mesh.surface_add_vertex(local_position)
		_mesh.surface_add_vertex(local_position + (DEBUG_PARTICLE_LENGTH * particle.tangent))

		# Normal line
		_mesh.surface_add_vertex(local_position)
		_mesh.surface_add_vertex(local_position + (DEBUG_PARTICLE_LENGTH * particle.normal))

		# Binormal line
		_mesh.surface_add_vertex(local_position)
		_mesh.surface_add_vertex(local_position + (DEBUG_PARTICLE_LENGTH * particle.binormal))

	_mesh.surface_end()

func _on_screen_entered():
	print("entered screen")
	draw = true

func _on_screen_exited():
	print("exited screen")
	draw = false

# Attaches a Node3D object to follow the end of the rope
func attach_object_to_end(object: Node3D) -> void:
	if object == null:
		return
		
	_payload_object = object
	print("Object attached to rope end: ", object.name)

# Removes the attached object from following the rope end
func detach_object_from_end() -> void:
	if _payload_object != null:
		print("Object detached from rope end: ", _payload_object.name)
		_payload_object = null

# Updates the position of the object attached to the rope end during gameplay
func update_attached_objects() -> void:
	if Engine.is_editor_hint() or _particle_data == null or (_payload_object == null and _editor_payload_object == null):
		return
	
	# Get the end particle position and orientation
	var end_index = simulation_particles - 1
	var end_particle = _particle_data.particles[end_index]
	var end_position = end_particle.position_current
	
	# Update programmatically attached object
	if is_instance_valid(_payload_object):
		_payload_object.global_position = end_position
		
		# Update rotation if enabled
		if rotate_payload_object:
			_update_object_rotation(_payload_object, end_particle)
		
	# Update object attached via editor
	if is_instance_valid(_editor_payload_object):
		_editor_payload_object.global_position = end_position
		
		# Update rotation if enabled
		if rotate_payload_object:
			_update_object_rotation(_editor_payload_object, end_particle)

# Updates attached object specifically in editor mode
func update_attached_object_editor() -> void:
	if not Engine.is_editor_hint() or _particle_data == null:
		return
	
	# Always refresh editor object reference
	_editor_payload_object = null
	if not payload_object.is_empty():
		var node = get_node_or_null(payload_object)
		if node is Node3D:
			_editor_payload_object = node
	
	# If we have an object, update its position
	if is_instance_valid(_editor_payload_object):
		var end_index = simulation_particles - 1
		var end_particle = _particle_data.particles[end_index]
		_editor_payload_object.global_position = end_particle.position_current
		
		# Update rotation if enabled
		if rotate_payload_object:
			_update_object_rotation(_editor_payload_object, end_particle)

# Helper to update an object's rotation to match rope end orientation
func _update_object_rotation(object: Node3D, particle) -> void:
	# Create a basis using the particle's orientation vectors
	var basis = Basis(particle.tangent, particle.binormal, particle.normal)
	
	# Convert to a quaternion and set the object's rotation
	object.global_transform.basis = basis
