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

	# 1b. Entrées invalides : erreur Godot propre, dictionnaire vide, pas de plantage
	var bad = ClassDB.instantiate("LodBuilder")
	bad.ratio = 2.0
	_check(bad.build(arrays[Mesh.ARRAY_VERTEX], arrays[Mesh.ARRAY_INDEX]).is_empty(), "ratio = 2.0 refusé sans plantage")
	bad.defaults()
	_check(bad.build(arrays[Mesh.ARRAY_VERTEX], PackedInt32Array([0, 1, 99999])).is_empty(), "indice hors plage refusé")
	_check(bad.build(arrays[Mesh.ARRAY_VERTEX], PackedInt32Array([0, -1, 2])).is_empty(), "indice négatif refusé")
	_check(bad.build(PackedVector3Array(), PackedInt32Array()).is_empty(), "maillage vide refusé")
	_check(not bad.build(arrays[Mesh.ARRAY_VERTEX], arrays[Mesh.ARRAY_INDEX]).is_empty(), "le même builder reconstruit ensuite")

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
	# 2b. add_cell après begin_frame : ajout incrémental, seule la nouvelle cellule est évaluée
	var third: int = sel.add_cell(chain, Vector3(0, 0, -300), 5.0)
	sel.begin_frame(Vector3(0, 0, -0.5), 70.0, 1080.0)
	lv = sel.select_changed(2.0)
	_check(third == 2 and lv[near] == -1 and lv[far] == -1 and lv[third] >= 0 and sel.evaluated() == 1, "add_cell incrémental : 1 seule évaluation (%d)" % sel.evaluated())
	_check(sel.add_cell(42, Vector3.ZERO, 1.0) == -1 and sel.cell_count() == 3, "chaîne inconnue refusée")
	_check(sel.add_chain(PackedFloat32Array([0.0, 0.5, 0.2])) == -1, "erreurs non croissantes refusées")
	_check(sel.select_changed(0.0).is_empty(), "seuil 0 px refusé")
	# 2c. hystérésis : caméra oscillant de ±2 cm autour du seuil du niveau 1 de la
	# cellule « près » (centre z = −20, rayon 5) : d_1 = 0,05·k/2 ⇒ z = −(25 + d_1).
	# Sans hystérésis le niveau bascule à chaque image ; avec, plus jamais.
	var k := 1080.0 / (2.0 * tan(deg_to_rad(35.0)))
	var edge := 20.0 + 5.0 + 0.05 * k / 2.0
	var flips := [0, 0]
	var seen := [false, false]
	for run in 2:
		sel.hysteresis = 0.0 if run == 0 else 0.15
		var last_level := -2
		for f in 200:
			var z := -edge + (0.02 if f % 2 == 0 else -0.02)
			sel.begin_frame(Vector3(0, 0, z), 70.0, 1080.0)
			var v: PackedInt32Array = sel.select_changed(2.0)
			if v[near] >= 0:
				seen[run] = true
				if last_level != -2 and v[near] != last_level:
					flips[run] += 1
				last_level = v[near]
	_check(seen[0] and flips[0] > 100, "sans hystérésis : la cellule clignote autour du seuil (%d bascules)" % flips[0])
	_check(flips[1] == 0, "hystérésis : aucune bascule en oscillant autour d'un seuil (%d)" % flips[1])

	# 2d. remove_cell : l'id libéré n'est plus évalué, cell_count décroît, le
	# prochain add_cell réemploie l'emplacement et est réévalué.
	_check(sel.remove_cell(third) and sel.cell_count() == 2, "remove_cell : cell_count 3 → 2 (%d)" % sel.cell_count())
	_check(not sel.remove_cell(third) and not sel.remove_cell(42), "double retrait / id inconnu refusés sans plantage")
	sel.begin_frame(Vector3(0, 0, -0.5), 70.0, 1080.0)
	lv = sel.select_changed(2.0)
	_check(lv.size() == 3 and lv[third] == -1, "cellule retirée : −1 dans select_changed")
	var reused: int = sel.add_cell(chain, Vector3(0, 0, -300), 5.0)
	sel.begin_frame(Vector3(0, 0, -0.5), 70.0, 1080.0)
	lv = sel.select_changed(2.0)
	_check(reused == third and lv[reused] >= 0 and sel.cell_count() == 3, "emplacement réemployé (%d) et réévalué (%d)" % [reused, lv[reused]])

	# 3. Nœud LodCells : une scène avec la grille, 6 exemplaires sur 2 cellules
	var node: Node3D = LodCellsScript.new()
	node.cell_size = 100.0
	node.target_error = 1.0
	node.cache_dir = ""
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

	# 4. Les exports du nœud atteignent LodBuilder : max_levels = 1 ⇒ moins de MultiMesh
	var node1: Node3D = LodCellsScript.new()
	node1.cell_size = 100.0
	node1.target_error = 1.0
	node1.max_levels = 1
	node1.cache_dir = ""
	get_root().add_child(node1)
	node1.add_building(scene, xforms)
	_check(node1.multimesh_count == 2 * 2 and node1.multimesh_count < node.multimesh_count, "max_levels=1 : %d MultiMesh vs %d avec 4" % [node1.multimesh_count, node.multimesh_count])
	var node2: Node3D = LodCellsScript.new()
	node2.cell_size = 100.0
	node2.target_error = 1.0
	node2.ratio = 2.0
	node2.cache_dir = ""
	get_root().add_child(node2)
	node2.add_building(scene, xforms)
	_check(node2.multimesh_count == 2 and node2.instance_count == 6, "ratio invalide : repli à un niveau, pas de plantage (%d)" % node2.multimesh_count)

	# 4b. Tampon d'instances (partagé par les niveaux) : matrice 3×4 en lignes,
	# relative à l'ancre, composée avec la transformation de la pièce.
	var xr := Transform3D(Basis(Vector3(0.3, 1, 0.2).normalized(), 1.1).scaled(Vector3(2, 2, 2)), Vector3(100, 5, -40))
	var loc := Transform3D(Basis(Vector3.UP, 0.4), Vector3(1, 2, 3))
	var anchor := Vector3(90, 0, -50)
	var buf: PackedFloat32Array = LodCellsScript.instance_buffer([xr, Transform3D.IDENTITY], loc, anchor)
	var back := Transform3D(Basis(Vector3(buf[0], buf[4], buf[8]), Vector3(buf[1], buf[5], buf[9]), Vector3(buf[2], buf[6], buf[10])), Vector3(buf[3], buf[7], buf[11]))
	_check(buf.size() == 24 and back.is_equal_approx(Transform3D(xr.basis, xr.origin - anchor) * loc), "instance_buffer : 12 floats par exemplaire, lignes de (exemplaire − ancre) × pièce")

	# 5. add_cell / clear_cell : deux clés, retrait de l'une, MMI libérés,
	# compteurs décroissants, réemploi des emplacements du sélecteur.
	var node5: Node3D = LodCellsScript.new()
	node5.cell_size = 100.0
	node5.target_error = 1.0
	node5.cache_dir = ""
	get_root().add_child(node5)
	var xa: Array = xforms.slice(0, 3)   # une tuile
	var xb: Array = xforms.slice(3, 6)   # l'autre tuile
	var created_a: int = node5.add_cell(Vector2i(0, 0), scene, xa)
	var created_b: int = node5.add_cell(Vector2i(0, 1), scene, xb)
	var n_mmi_full := node5.get_child_count()
	_check(created_a == 1 and created_b == 1 and node5.cell_count == 2 and node5.instance_count == 6, "add_cell : 2 clés → 2 cellules, 6 exemplaires (%d, %d)" % [node5.cell_count, node5.instance_count])
	_check(node5.has_cell(Vector2i(0, 0)) and not node5.has_cell(Vector2i(9, 9)) and node5.cell_keys().size() == 2, "has_cell / cell_keys")
	_check(node5.clear_cell(Vector2i(0, 0)) and not node5.clear_cell(Vector2i(0, 0)), "clear_cell : vrai une fois, faux ensuite")
	_check(node5.cell_count == 1 and node5.instance_count == 3 and node5.multimesh_count == levels.size(), "après clear : 1 cellule, 3 exemplaires, %d MultiMesh (%d)" % [levels.size(), node5.multimesh_count])
	_check(node5.get_child_count() == n_mmi_full - levels.size(), "MultiMeshInstance3D libérés de l'arbre (%d → %d)" % [n_mmi_full, node5.get_child_count()])
	_check(node5._selector.cell_count() == 1 and node5.levels().size() == 1, "sélecteur : 1 cellule vivante, levels() sur les vivantes")
	node5.add_cell(Vector2i(0, 0), scene, xa)
	_check(node5.cell_count == 2 and node5._cells.size() == 2 and node5.get_child_count() == n_mmi_full, "ré-ajout : emplacement réemployé, pas de croissance (%d emplacements)" % node5._cells.size())
	_check(node5.clear_cell(Vector2i(0, 0)) and node5.clear_cell(Vector2i(0, 1)) and node5.cell_count == 0 and node5.instance_count == 0 and node5.multimesh_count == 0 and node5.get_child_count() == 0, "tout retiré : compteurs à 0, aucun enfant")

	# 6. Cache disque : écrit au premier build, relu au second (aucun LodBuilder
	# appelé : cache_hits), octets identiques après ré-encodage, invalidé par version.
	var dir := "user://lodcache-test"
	var abs_dir := ProjectSettings.globalize_path(dir)
	if DirAccess.dir_exists_absolute(abs_dir):
		for f in DirAccess.get_files_at(abs_dir):
			DirAccess.remove_absolute(abs_dir.path_join(f))
	var node6: Node3D = LodCellsScript.new()
	node6.cell_size = 100.0
	node6.target_error = 1.0
	node6.cache_dir = dir
	get_root().add_child(node6)
	node6.add_building(scene, xforms)
	var files := DirAccess.get_files_at(abs_dir)
	_check(node6.cache_misses == 1 and node6.cache_hits == 0 and files.size() == 1 and files[0].ends_with(".bin"), "premier build : 1 fichier de cache écrit, 1 miss (%s)" % [files])
	var cache_path := dir.path_join(files[0])
	var bytes := FileAccess.get_file_as_bytes(cache_path)
	var desc: Dictionary = LodCellsScript.decode_cache(bytes)
	_check(not desc.is_empty() and LodCellsScript.encode_cache(desc) == bytes, "relu puis ré-encodé : identique octet à octet (%d octets)" % bytes.size())
	_check((desc["levels"] as Array).size() == levels.size() and desc["errors"] == errors, "le cache porte les mêmes niveaux et erreurs que LodBuilder")
	var node7: Node3D = LodCellsScript.new()
	node7.cell_size = 100.0
	node7.target_error = 1.0
	node7.cache_dir = dir
	get_root().add_child(node7)
	node7.add_building(scene, xforms)
	_check(node7.cache_hits == 1 and node7.cache_misses == 0 and node7.multimesh_count == node6.multimesh_count and node7.lod_mesh_count == node6.lod_mesh_count, "second nœud : relu du cache (hit), mêmes MultiMesh (%d)" % node7.multimesh_count)
	var am7: ArrayMesh = node7._lod_cache[grid.get_rid()]["meshes"][levels.size() - 1]
	var am6: ArrayMesh = node6._lod_cache[grid.get_rid()]["meshes"][levels.size() - 1]
	_check(am7.surface_get_arrays(0)[Mesh.ARRAY_INDEX] == am6.surface_get_arrays(0)[Mesh.ARRAY_INDEX], "dernier niveau identique construit / relu")
	# version : on altère l'en-tête du fichier → ignoré, reconstruit et réécrit
	var bad_bytes := bytes.duplicate()
	bad_bytes.encode_u32(4, LodCellsScript.CACHE_VERSION + 1)
	var fw := FileAccess.open(cache_path, FileAccess.WRITE)
	fw.store_buffer(bad_bytes)
	fw.close()
	_check(LodCellsScript.decode_cache(bad_bytes).is_empty(), "autre version : décodage refusé")
	var node8: Node3D = LodCellsScript.new()
	node8.cell_size = 100.0
	node8.target_error = 1.0
	node8.cache_dir = dir
	get_root().add_child(node8)
	node8.add_building(scene, xforms)
	_check(node8.cache_misses == 1 and FileAccess.get_file_as_bytes(cache_path) == bytes, "version périmée : reconstruit et fichier réécrit à l'identique")
	# paramètres différents ⇒ autre clé
	_check(LodCellsScript.cache_key([arrays], {"ratio": 0.5, "target_error": 1.0, "weld_tolerance": 0.0001, "max_levels": 4}) != LodCellsScript.cache_key([arrays], {"ratio": 0.5, "target_error": 1.0, "weld_tolerance": 0.0001, "max_levels": 2}), "la clé dépend des paramètres")

	# 7. Construction hors fil principal : prepare_scene lance la tâche, reap()
	# la récolte, add_cell n'a plus rien à construire (pas de LodBuilder sur le
	# fil principal) ; un autre maillage, même scène.
	var grid2 := _grid_mesh(40)
	var scene2 := Node3D.new()
	var mi2 := MeshInstance3D.new()
	mi2.mesh = grid2
	scene2.add_child(mi2)
	var node9: Node3D = LodCellsScript.new()
	node9.cell_size = 100.0
	node9.target_error = 1.0
	node9.cache_dir = ""
	get_root().add_child(node9)
	var ready0: bool = node9.prepare_scene(scene2)
	var waited := 0
	while node9.reap() > 0 and waited < 20000:
		OS.delay_msec(1)
		waited += 1
	_check(not ready0 and node9.pending_count() == 0 and node9.prepare_scene(scene2), "prepare_scene : pas prêt, puis prêt après reap (%d ms)" % waited)
	var t0 := Time.get_ticks_usec()
	node9.add_cell(Vector2i(0, 0), scene2, xa)
	var add_us := Time.get_ticks_usec() - t0
	var t1 := Time.get_ticks_usec()
	var node10: Node3D = LodCellsScript.new()
	node10.cell_size = 100.0
	node10.target_error = 1.0
	node10.cache_dir = ""
	get_root().add_child(node10)
	node10.add_cell(Vector2i(0, 0), scene2, xa)
	var sync_us := Time.get_ticks_usec() - t1
	_check(node9.multimesh_count == node10.multimesh_count and node9.multimesh_count > 1, "niveaux identiques préparés en fil / construits en synchrone (%d MultiMesh)" % node9.multimesh_count)
	print("  add_cell après préparation : %.2f ms ; synchrone : %.2f ms" % [add_us / 1000.0, sync_us / 1000.0])
	_check(add_us < sync_us, "add_cell préparé plus rapide que la construction synchrone")
	scene2.free()

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
