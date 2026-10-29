extends SceneTree
## Tests headless de l'addon : godot --headless --path godot --script res://tests/test_lodcells.gd

const LodCellsScript := preload("res://addons/lodcells/lod_cells.gd")

var _fails := 0


func _init() -> void:
	_check(ClassDB.class_exists("LodBuilder") and ClassDB.class_exists("LodSelector"), "GDExtension chargée")
	if not ClassDB.class_exists("LodBuilder"):
		quit(1); return

	# 1. LodBuilder sur une grille plane 24×24 (2 surfaces identiques dans un ArrayMesh)
	var grid := _grid_mesh(24)
	var builder = ClassDB.instantiate("LodBuilder")
	builder.defaults()
	builder.target_error = 1.0
	var arrays := grid.surface_get_arrays(0)
	var r: Dictionary = builder.build(arrays[Mesh.ARRAY_VERTEX], arrays[Mesh.ARRAY_INDEX])
	var levels: Array = r["levels"]
	_check(levels.size() >= 3, "au moins 3 niveaux (%d)" % levels.size())
	var tris: PackedInt32Array = r["triangles"]
	var errors: PackedFloat32Array = r["errors"]
	var ok := true
	for i in range(1, levels.size()):
		ok = ok and tris[i] < tris[i - 1] and errors[i] > errors[i - 1]
		ok = ok and (levels[i] as PackedInt32Array).size() == tris[i] * 3
	_check(ok, "niveaux plus grossiers, erreurs croissantes, indices cohérents")
	var n_verts: int = (arrays[Mesh.ARRAY_VERTEX] as PackedVector3Array).size()
	var max_idx := 0
	for i in (levels[levels.size() - 1] as PackedInt32Array):
		max_idx = maxi(max_idx, i)
	_check(max_idx < n_verts, "indices dans le tampon de sommets d'origine")
	_check(float(r["radius"]) > 10.0, "rayon englobant plausible (%.1f)" % float(r["radius"]))

	# 2. LodSelector : deux cellules, près / loin
	var sel = ClassDB.instantiate("LodSelector")
	var chain: int = sel.add_chain(PackedFloat32Array([0.0, 0.05, 0.2, 0.8]))
	var near: int = sel.add_cell(chain, Vector3(0, 0, -20), 5.0)
	var far: int = sel.add_cell(chain, Vector3(0, 0, -2000), 5.0)
	sel.begin_frame(Vector3.ZERO, 70.0, 1080.0)
	var lv: PackedInt32Array = sel.select_changed(2.0)
	_check(lv[near] == 0 and lv[far] == 3, "près = niveau 0, loin = niveau 3 (%s)" % lv)
	sel.begin_frame(Vector3(0, 0, -0.5), 70.0, 1080.0)
	lv = sel.select_changed(2.0)
	_check(lv[near] == -1 and lv[far] == -1 and sel.evaluated() == 0, "petit déplacement : rien réévalué (slack)")

	# 3. Nœud LodCells : une scène avec la grille, 6 exemplaires sur 2 cellules
	var node: Node3D = LodCellsScript.new()
	node.cell_size = 100.0
	node.target_error = 1.0
	get_root().add_child(node)
	var scene := Node3D.new()
	var mi := MeshInstance3D.new()
	mi.mesh = grid
	scene.add_child(mi)
	var xforms: Array = []
	for i in 6:
		xforms.append(Transform3D(Basis(), Vector3(10.0 + (i % 3) * 20.0, 0, (i / 3) * 150.0)))
	node.add_building(scene, xforms)
	_check(node.instance_count == 6 and node.cell_count == 2, "6 exemplaires, 2 cellules (%d, %d)" % [node.instance_count, node.cell_count])
	_check(node.multimesh_count == 2 * levels.size(), "un MultiMesh par niveau et par cellule (%d)" % node.multimesh_count)
	_check(node.lod_mesh_count == levels.size() - 1, "maillages de LOD construits une fois (%d)" % node.lod_mesh_count)
	var am: ArrayMesh = node._lod_cache[grid.get_rid()]["meshes"][1]
	_check(am.get_surface_count() == 1 and am.surface_get_arrays(0)[Mesh.ARRAY_NORMAL] != null, "les niveaux gardent les attributs (normales)")

	scene.free()
	if _fails == 0:
		print("TEST LODCELLS OK")
	quit(0 if _fails == 0 else 1)


## Grille plane n×n avec normales.
static func _grid_mesh(n: int) -> ArrayMesh:
	var verts := PackedVector3Array()
	var normals := PackedVector3Array()
	var idx := PackedInt32Array()
	for z in n:
		for x in n:
			verts.append(Vector3(x, 0, z))
			normals.append(Vector3.UP)
	for z in n - 1:
		for x in n - 1:
			var a := z * n + x
			idx.append_array([a, a + n, a + 1, a + 1, a + n, a + n + 1])
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_INDEX] = idx
	var am := ArrayMesh.new()
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return am


func _check(ok: bool, what: String) -> void:
	if ok:
		print("  ok   ", what)
	else:
		_fails += 1
		printerr("  FAIL ", what)
