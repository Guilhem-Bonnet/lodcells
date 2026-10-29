extends Node3D
## LodCells : exemplaires statiques en MultiMesh par (maillage, niveau de LOD,
## cellule). Les niveaux sont construits par la GDExtension (lodcells-core :
## simplification à erreur additive monotone) en conservant les sommets
## d'origine — donc normales, UV et matériaux. Le niveau de chaque cellule
## est choisi en Rust d'après l'erreur projetée en pixels de sa sphère
## englobante, avec sélection incrémentale (« slack ») : une caméra immobile
## ne coûte rien.
##
## API volontairement identique à un regroupement MultiMesh simple :
##   add_building(scene: Node3D, xforms: Array[Transform3D])
## Le nœud peut être déplacé (décalage d'origine) : les cellules sont en
## coordonnées locales.

## Taille de cellule (m). Même pas qu'un éventuel décalage d'origine.
@export var cell_size := 409.6
## Au-delà : cellule masquée (visibility_range_end).
@export var visibility_end := 3000.0
## Seuil d'erreur projetée (pixels) : plus petit = plus fin, plus de triangles.
@export var threshold_px := 2.0
@export var max_levels := 4
@export var ratio := 0.5
@export var target_error := 0.05

var instance_count := 0
var multimesh_count := 0
var lod_mesh_count := 0
var cell_count := 0

var _builder: RefCounted
var _selector: RefCounted
var _lod_cache := {}   # RID du maillage → {"meshes": [Mesh…], "errors": PackedFloat32Array, "radius": float, "chain": int}
var _cells := []       # id de cellule → {"mmis": [MultiMeshInstance3D…], "level": int}


func _init() -> void:
	if ClassDB.class_exists("LodBuilder"):
		_builder = ClassDB.instantiate("LodBuilder")
		_builder.defaults()
		_selector = ClassDB.instantiate("LodSelector")
	else:
		push_warning("lodcells : GDExtension absente (tools/build.sh) — un seul niveau, pas de sélection")


static func available() -> bool:
	return ClassDB.class_exists("LodBuilder")


## Ajoute tous les exemplaires (`xforms`, repère de ce nœud) d'une scène GLB.
func add_building(scene: Node3D, xforms: Array) -> void:
	var parts := _parts(scene)
	if parts.is_empty() or xforms.is_empty():
		return
	var by_cell := {}
	for x: Transform3D in xforms:
		var key := Vector2i(floori(x.origin.x / cell_size), floori(x.origin.z / cell_size))
		if not by_cell.has(key):
			by_cell[key] = []
		by_cell[key].append(x)
	for key in by_cell:
		var list: Array = by_cell[key]
		for p: Dictionary in parts:
			var lod := _lods_for(p["mesh"])
			var local: Transform3D = p["xform"]
			var scale := maxf(local.basis.x.length(), maxf(local.basis.y.length(), local.basis.z.length()))
			# sphère englobante de la cellule : centroïde des exemplaires, rayon
			# = distance max au centroïde + rayon du maillage à l'échelle
			var centroid := Vector3.ZERO
			for x: Transform3D in list:
				centroid += (x * local).origin
			centroid /= list.size()
			var reach := 0.0
			for x: Transform3D in list:
				reach = maxf(reach, (x * local).origin.distance_to(centroid))
			var radius: float = reach + float(lod["radius"]) * scale
			var mmis: Array = []
			var meshes: Array = lod["meshes"]
			for level in meshes.size():
				var mm := MultiMesh.new()
				mm.transform_format = MultiMesh.TRANSFORM_3D
				mm.mesh = meshes[level]
				mm.instance_count = list.size()
				for i in list.size():
					mm.set_instance_transform(i, list[i] * local)
				var mmi := MultiMeshInstance3D.new()
				mmi.multimesh = mm
				mmi.visibility_range_end = visibility_end
				mmi.visible = level == 0
				add_child(mmi)
				mmis.append(mmi)
				multimesh_count += 1
			var cell_id: int = _selector.add_cell(int(lod["chain"]), centroid, radius) if _selector != null else _cells.size()
			_cells.append({"mmis": mmis, "level": 0})
			assert(cell_id == _cells.size() - 1)
			cell_count += 1
		instance_count += list.size()


## Niveaux de LOD d'un maillage (construits une fois, tous attributs conservés).
func _lods_for(mesh: Mesh) -> Dictionary:
	var key := mesh.get_rid()
	if _lod_cache.has(key):
		return _lod_cache[key]
	var result := {"meshes": [mesh], "errors": PackedFloat32Array([0.0]), "radius": 0.0, "chain": -1}
	if _builder == null:
		_lod_cache[key] = result
		return result
	var per_surface: Array = []
	var n_levels := 1 << 30
	var radius := 0.0
	for s in mesh.get_surface_count():
		var arrays := mesh.surface_get_arrays(s)
		var pos: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
		var idx: Variant = arrays[Mesh.ARRAY_INDEX]
		var r: Dictionary = _builder.build(pos, idx if idx != null else PackedInt32Array())
		per_surface.append({"arrays": arrays, "r": r})
		n_levels = mini(n_levels, (r["levels"] as Array).size())
		radius = maxf(radius, float(r["radius"]) + (r["center"] as Vector3).length())
	result["radius"] = radius
	if n_levels <= 1:
		_lod_cache[key] = result
		return result
	# erreur d'un niveau = max sur les surfaces (toutes basculent ensemble)
	var errors := PackedFloat32Array()
	errors.resize(n_levels)
	for level in n_levels:
		var e := 0.0
		for ps in per_surface:
			e = maxf(e, float(ps["r"]["errors"][level]))
		errors[level] = e
	var meshes: Array = [mesh]
	for level in range(1, n_levels):
		var am := ArrayMesh.new()
		for s in per_surface.size():
			var arrays: Array = per_surface[s]["arrays"].duplicate()
			arrays[Mesh.ARRAY_INDEX] = per_surface[s]["r"]["levels"][level]
			am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
			am.surface_set_material(s, mesh.surface_get_material(s))
		meshes.append(am)
		lod_mesh_count += 1
	result["meshes"] = meshes
	result["errors"] = errors
	result["chain"] = _selector.add_chain(errors)
	_lod_cache[key] = result
	return result


func _process(_delta: float) -> void:
	if _selector == null or _cells.is_empty():
		return
	var cam := get_viewport().get_camera_3d()
	if cam == null:
		return
	_selector.begin_frame(to_local(cam.global_position), cam.fov, get_viewport().get_visible_rect().size.y)
	var changed: PackedInt32Array = _selector.select_changed(threshold_px)
	for i in changed.size():
		var level := changed[i]
		if level < 0:
			continue
		var cell: Dictionary = _cells[i]
		var mmis: Array = cell["mmis"]
		level = mini(level, mmis.size() - 1)
		if level == cell["level"]:
			continue
		mmis[cell["level"]].visible = false
		mmis[level].visible = true
		cell["level"] = level


## Nombre de cellules réévaluées à la dernière image (diagnostic).
func evaluated_last_frame() -> int:
	return int(_selector.evaluated()) if _selector != null else 0


## Niveau courant de chaque cellule (diagnostic / tests).
func levels() -> PackedInt32Array:
	var out := PackedInt32Array()
	for c in _cells:
		out.append(int(c["level"]))
	return out


## Maillages d'une scène avec leur transformation complète, racine comprise.
static func _parts(scene: Node3D) -> Array:
	var out := []
	for mi: MeshInstance3D in scene.find_children("*", "MeshInstance3D", true, false):
		if mi.mesh == null:
			continue
		var x := mi.transform
		var n := mi.get_parent()
		while n != null and n != scene and n is Node3D:
			x = (n as Node3D).transform * x
			n = n.get_parent()
		out.append({"mesh": mi.mesh, "xform": scene.transform * x})
	return out
