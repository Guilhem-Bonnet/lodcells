extends Node3D
## LodCells : exemplaires statiques en MultiMesh par (maillage, niveau de LOD,
## cellule). Les niveaux sont construits par la GDExtension (lodcells-core :
## simplification à erreur additive monotone) en conservant les sommets
## d'origine — donc normales, UV et matériaux. Le niveau de chaque cellule
## est choisi en Rust d'après l'erreur projetée en pixels de sa sphère
## englobante, avec sélection incrémentale (« slack ») : une caméra immobile
## ne coûte rien.
##
## API (S06, incrémentale) :
##   add_cell(key, scene: Node3D, xforms: Array[Transform3D], vis_end := -1.0)
##       ajoute les exemplaires d'une scène sous une clé (ex. Vector2i de
##       cellule de streaming) ; plusieurs scènes peuvent partager une clé.
##   clear_cell(key) -> bool   libère les MultiMeshInstance3D de la clé,
##       cell_count / instance_count / multimesh_count décroissent.
##   add_building(scene, xforms, vis_end)   = add_cell sous une clé automatique
##       (entiers négatifs), conservée pour compatibilité.
##   prepare_scene(scene) -> bool   lance en WorkerThreadPool la construction
##       des niveaux de LOD des maillages de la scène (LodBuilder.build est du
##       Rust pur ; chaque tâche instancie son propre LodBuilder) ; renvoie true
##       quand tout est prêt. reap() libère les tâches finies (à appeler
##       régulièrement) et renvoie le nombre encore en cours. Sans préparation,
##       add_cell construit en synchrone sur le fil principal.
##   Cache disque des niveaux : user://lodcache/<md5 du maillage>.bin
##       (en-tête "LODC" + version ; relu identique octet à octet, invalidé par
##       CACHE_VERSION ou par un changement des paramètres de construction).
## Le nœud peut être déplacé (décalage d'origine) : les cellules sont en
## coordonnées locales. Chaque MultiMeshInstance3D est ancrée au centroïde de
## sa cellule et ses exemplaires sont stockés relativement à cette ancre : loin
## de l'origine, le GPU ne manipule que de petits nombres.

## Version du format du cache disque : à incrémenter si le contenu change.
const CACHE_VERSION := 1
const CACHE_MAGIC := "LODC"

## Taille de cellule (m). Même pas qu'un éventuel décalage d'origine.
@export var cell_size := 409.6
## Au-delà : cellule masquée (visibility_range_end).
@export var visibility_end := 3000.0
## Seuil d'erreur projetée (pixels) : plus petit = plus fin, plus de triangles.
@export var threshold_px := 2.0
## Paramètres du constructeur de niveaux (poussés vers LodBuilder avant chaque
## construction : un changement après l'ajout n'affecte que les maillages
## pas encore construits ; ils font partie de la clé du cache disque).
@export var max_levels := 4
@export var ratio := 0.5
@export var target_error := 0.05
@export var weld_tolerance := 0.0001
## Marge d'hystérésis sur l'erreur projetée (0 = aucune) : évite le
## clignotement quand la caméra oscille autour d'un seuil.
@export var hysteresis := 0.15
## Dossier du cache disque des niveaux ("" = pas de cache).
@export var cache_dir := "user://lodcache"

var instance_count := 0
var multimesh_count := 0
var lod_mesh_count := 0
var cell_count := 0
## Diagnostic du cache disque.
var cache_hits := 0
var cache_misses := 0

var _selector: RefCounted
var _lod_cache := {}   # RID du maillage → {"meshes": [Mesh…], "errors": PackedFloat32Array, "center": Vector3, "radius": float}
var _chain_cache := {} # [RID, échelle] → chaîne du sélecteur (erreurs à l'échelle de la scène)
var _cells := []       # id de cellule → {"mmis": [MultiMeshInstance3D…], "level": int, "n": int} ou null (libérée)
var _groups := {}      # clé → [ids de cellules]
var _group_totals := {}  # clé → exemplaires ajoutés (retranchés par clear_cell)
var _auto_key := 0
var _mutex := Mutex.new()
var _pending := {}     # RID → id de tâche WorkerThreadPool
var _results := {}     # RID → résultat d'une tâche finie, à intégrer par reap()/_lods_for


func _init() -> void:
	if ClassDB.class_exists("LodSelector"):
		_selector = ClassDB.instantiate("LodSelector")
	else:
		push_warning("lodcells : GDExtension absente (tools/build.sh) — un seul niveau, pas de sélection")


func _exit_tree() -> void:
	# Chaque tâche doit être attendue une fois ; les résultats restent cachés.
	wait_all()


static func available() -> bool:
	return ClassDB.class_exists("LodBuilder")


## Ajoute tous les exemplaires (`xforms`, repère de ce nœud) d'une scène GLB
## sous une clé automatique (compatibilité). `vis_end` : portée de masquage (m)
## pour cette scène, `visibility_end` par défaut ; 0.0 = jamais masquée.
func add_building(scene: Node3D, xforms: Array, vis_end := -1.0) -> void:
	_auto_key -= 1
	add_cell(_auto_key, scene, xforms, vis_end)


## Ajoute les exemplaires d'une scène sous la clé `key` (Variant hashable :
## Vector2i de cellule, int, String…). Renvoie le nombre de cellules
## (maillage × tuile) créées.
func add_cell(key: Variant, scene: Node3D, xforms: Array, vis_end := -1.0) -> int:
	var parts := _parts(scene)
	if parts.is_empty() or xforms.is_empty():
		return 0
	if vis_end < 0.0:
		vis_end = visibility_end
	var by_cell := {}
	for x: Transform3D in xforms:
		var k := Vector2i(floori(x.origin.x / cell_size), floori(x.origin.z / cell_size))
		if not by_cell.has(k):
			by_cell[k] = []
		by_cell[k].append(x)
	var ids: Array = _groups.get(key, [])
	var created := 0
	for k in by_cell:
		var list: Array = by_cell[k]
		# ancre de la tuile : centroïde des origines des exemplaires (coordonnées
		# locales du nœud) ; les transformations d'instance sont relatives à elle
		var anchor := Vector3.ZERO
		for x: Transform3D in list:
			anchor += x.origin
		anchor /= list.size()
		for p: Dictionary in parts:
			var lod := _lods_for(p["mesh"])
			var local: Transform3D = p["xform"]
			var scale := maxf(local.basis.x.length(), maxf(local.basis.y.length(), local.basis.z.length()))
			# sphère englobante de la cellule : centroïde des centres de maillage
			# transformés, rayon = distance max au centroïde + rayon du maillage à
			# l'échelle (un maillage dont les sommets sont loin de son origine — une
			# tuile de terrain — garde ainsi une sphère juste).
			var center: Vector3 = lod["center"]
			var centroid := Vector3.ZERO
			for x: Transform3D in list:
				centroid += (x * local) * center
			centroid /= list.size()
			var reach := 0.0
			for x: Transform3D in list:
				reach = maxf(reach, ((x * local) * center).distance_to(centroid))
			var radius: float = reach + float(lod["radius"]) * scale
			var mmis: Array = []
			var meshes: Array = lod["meshes"]
			# Tampon d'instances construit UNE fois par pièce et partagé par tous les
			# niveaux (mêmes transformations) : 12 floats par exemplaire, matrice 3×4
			# en lignes (cf. MultiMesh.buffer), au lieu de set_instance_transform ×
			# niveaux.
			var buf := instance_buffer(list, local, anchor)
			for level in meshes.size():
				var mm := MultiMesh.new()
				mm.transform_format = MultiMesh.TRANSFORM_3D
				mm.mesh = meshes[level]
				mm.instance_count = list.size()
				mm.buffer = buf
				var mmi := MultiMeshInstance3D.new()
				mmi.multimesh = mm
				mmi.position = anchor
				mmi.visibility_range_end = vis_end
				mmi.visible = level == 0
				add_child(mmi)
				mmis.append(mmi)
				multimesh_count += 1
			var cell_id: int = _selector.add_cell(_chain_for(lod, scale), centroid, radius) if _selector != null else _free_slot()
			var cell := {"mmis": mmis, "level": 0, "n": list.size()}
			if cell_id < _cells.size():
				assert(_cells[cell_id] == null)
				_cells[cell_id] = cell
			else:
				assert(cell_id == _cells.size())
				_cells.append(cell)
			ids.append(cell_id)
			cell_count += 1
			created += 1
		instance_count += list.size()
		_group_totals[key] = int(_group_totals.get(key, 0)) + list.size()
	_groups[key] = ids
	return created


## Tampon MultiMesh (TRANSFORM_3D, sans couleur ni données) des exemplaires
## `xforms × local`, relatifs à `anchor` : par exemplaire les 3 lignes de la
## matrice 3×4 (basis.x.x, basis.y.x, basis.z.x, origin.x, …).
static func instance_buffer(xforms: Array, local: Transform3D, anchor: Vector3) -> PackedFloat32Array:
	var buf := PackedFloat32Array()
	buf.resize(xforms.size() * 12)
	var k := 0
	for x: Transform3D in xforms:
		var t := Transform3D(x.basis, x.origin - anchor) * local
		var b := t.basis
		buf[k] = b.x.x; buf[k + 1] = b.y.x; buf[k + 2] = b.z.x; buf[k + 3] = t.origin.x
		buf[k + 4] = b.x.y; buf[k + 5] = b.y.y; buf[k + 6] = b.z.y; buf[k + 7] = t.origin.y
		buf[k + 8] = b.x.z; buf[k + 9] = b.y.z; buf[k + 10] = b.z.z; buf[k + 11] = t.origin.z
		k += 12
	return buf


## Premier emplacement libre (sans sélecteur) : même réemploi que LodSelector.
func _free_slot() -> int:
	for i in _cells.size():
		if _cells[i] == null:
			return i
	return _cells.size()


func has_cell(key: Variant) -> bool:
	return _groups.has(key)


## Clés des cellules chargées.
func cell_keys() -> Array:
	return _groups.keys()


## Libère tous les MultiMeshInstance3D d'une clé ; false si la clé est inconnue.
func clear_cell(key: Variant) -> bool:
	if not _groups.has(key):
		return false
	for id: int in _groups[key]:
		var cell: Dictionary = _cells[id]
		for mmi: MultiMeshInstance3D in cell["mmis"]:
			remove_child(mmi)
			mmi.free()
			multimesh_count -= 1
		if _selector != null:
			_selector.remove_cell(id)
		_cells[id] = null
		cell_count -= 1
	# instance_count compte les exemplaires une fois par scène (pas par pièce)
	instance_count -= int(_group_totals[key])
	_groups.erase(key)
	_group_totals.erase(key)
	return true


# ------------------------------------------------------------ niveaux de LOD

## Lance en arrière-plan la construction des niveaux des maillages d'une scène
## non encore connus ; true si tout est déjà prêt.
## `budget_usec` > 0 : cesse de lancer des tâches une fois le budget dépassé
## (la lecture des tableaux de surface se fait sur le fil principal) ; le reste
## partira aux appels suivants.
func prepare_scene(scene: Node3D, budget_usec := 0) -> bool:
	var t0 := Time.get_ticks_usec()
	var ready := true
	for p: Dictionary in _parts(scene):
		if _lod_cache.has((p["mesh"] as Mesh).get_rid()):
			continue
		ready = false
		if budget_usec > 0 and Time.get_ticks_usec() - t0 > budget_usec:
			break
		prepare_mesh(p["mesh"])
	return ready


## Idem pour un maillage seul.
func prepare_mesh(mesh: Mesh) -> bool:
	var key := mesh.get_rid()
	if _lod_cache.has(key):
		return true
	_mutex.lock()
	var done: bool = _results.has(key)
	var pending: bool = _pending.has(key)
	_mutex.unlock()
	if done:
		reap()
		return true
	if not pending:
		var params := _params()
		_mutex.lock()
		# Les tableaux de surface sont lus ICI (fil principal) : surface_get_arrays
		# est un appel synchronisé du RenderingServer, qui depuis un fil de travail
		# attend la fin de l'image courante (blocage si le fil principal attend).
		_pending[key] = WorkerThreadPool.add_task(_build_task.bind(mesh, params, _surfaces_of(mesh)), false, "lod " + str(key.get_id()))
		_mutex.unlock()
	return false


## Libère les tâches de fond terminées et intègre leurs résultats ; renvoie le
## nombre de tâches encore en cours.
func reap() -> int:
	_mutex.lock()
	for key in _pending.keys():
		if WorkerThreadPool.is_task_completed(_pending[key]):
			WorkerThreadPool.wait_for_task_completion(_pending[key])
			_pending.erase(key)
	for key in _results.keys():
		_store(key, _results[key])
	_results.clear()
	var left := _pending.size()
	_mutex.unlock()
	return left


## Attend toutes les tâches en cours (sortie, tests).
func wait_all() -> void:
	_mutex.lock()
	var tasks: Array = _pending.values()
	_pending.clear()
	_mutex.unlock()
	for t in tasks:
		WorkerThreadPool.wait_for_task_completion(t)
	reap()


func pending_count() -> int:
	_mutex.lock()
	var n := _pending.size()
	_mutex.unlock()
	return n


func _store(key: RID, result: Dictionary) -> void:
	if _lod_cache.has(key):
		return
	_lod_cache[key] = result
	lod_mesh_count += (result["meshes"] as Array).size() - 1
	if result.get("from_cache", false):
		cache_hits += 1
	elif result.get("built", false):
		cache_misses += 1


## Niveaux de LOD d'un maillage (construits une fois, tous attributs conservés).
## Attend une tâche de fond en cours ; sinon construit en synchrone.
func _lods_for(mesh: Mesh) -> Dictionary:
	var key := mesh.get_rid()
	if _lod_cache.has(key):
		return _lod_cache[key]
	_mutex.lock()
	var task: int = _pending.get(key, -1)
	_mutex.unlock()
	if task >= 0:
		WorkerThreadPool.wait_for_task_completion(task)
		_mutex.lock()
		_pending.erase(key)
		_mutex.unlock()
		reap()
		if _lod_cache.has(key):
			return _lod_cache[key]
	_build_task(mesh, _params(), _surfaces_of(mesh))
	reap()
	return _lod_cache[key]


## Tableaux de toutes les surfaces d'un maillage (à lire sur le fil principal).
static func _surfaces_of(mesh: Mesh) -> Array:
	var surfaces: Array = []
	for s in mesh.get_surface_count():
		surfaces.append(mesh.surface_get_arrays(s))
	return surfaces


func _params() -> Dictionary:
	return {"max_levels": max_levels, "ratio": ratio, "target_error": target_error, "weld_tolerance": weld_tolerance, "cache_dir": cache_dir}


## Construit (ou relit du cache disque) les niveaux d'un maillage. Appelable
## depuis un fil de travail : LodBuilder local à la tâche, ArrayMesh créés hors
## arbre (appels RenderingServer sans retour, donc sans synchronisation),
## résultat déposé sous mutex. `surfaces` : tableaux lus sur le fil principal.
func _build_task(mesh: Mesh, params: Dictionary, surfaces: Array) -> void:
	var key := mesh.get_rid()
	var aabb := mesh.get_aabb()
	var result := {"meshes": [mesh], "errors": PackedFloat32Array([0.0]), "center": aabb.get_center(), "radius": aabb.size.length() * 0.5}
	if available() and not surfaces.is_empty():
		var desc := _load_or_build(surfaces, params)
		if not desc.is_empty():
			result["from_cache"] = desc["from_cache"]
			result["built"] = not desc["from_cache"]
			var n_levels: int = (desc["levels"] as Array).size()
			if n_levels > 1:
				result["errors"] = desc["errors"]
				result["center"] = desc["center"]
				result["radius"] = desc["radius"]
				var meshes: Array = [mesh]
				for level in range(1, n_levels):
					var am := ArrayMesh.new()
					var per_surface: Array = desc["levels"][level]
					for s in surfaces.size():
						var arrays: Array = surfaces[s].duplicate()
						arrays[Mesh.ARRAY_INDEX] = per_surface[s]
						am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
						am.surface_set_material(s, mesh.surface_get_material(s))
					meshes.append(am)
				result["meshes"] = meshes
	_mutex.lock()
	_results[key] = result
	_mutex.unlock()


## Descripteur des niveaux (indices par niveau et par surface, erreurs, sphère),
## relu du cache disque ou construit puis écrit. Vide si la GDExtension refuse
## l'entrée (déjà signalé).
func _load_or_build(surfaces: Array, params: Dictionary) -> Dictionary:
	var path := ""
	if String(params["cache_dir"]) != "":
		path = String(params["cache_dir"]).path_join(cache_key(surfaces, params) + ".bin")
		var cached := read_cache_file(path)
		if not cached.is_empty():
			cached["from_cache"] = true
			return cached
	var builder = ClassDB.instantiate("LodBuilder")
	builder.max_levels = params["max_levels"]
	builder.ratio = params["ratio"]
	builder.target_error = params["target_error"]
	builder.weld_tolerance = params["weld_tolerance"]
	var per_surface: Array = []
	var n_levels := 1 << 30
	for arrays: Array in surfaces:
		var idx: Variant = arrays[Mesh.ARRAY_INDEX]
		var r: Dictionary = builder.build(arrays[Mesh.ARRAY_VERTEX], idx if idx != null else PackedInt32Array())
		if r.is_empty():
			return {}
		per_surface.append(r)
		n_levels = mini(n_levels, (r["levels"] as Array).size())
	# erreur d'un niveau = max sur les surfaces (toutes basculent ensemble)
	var errors := PackedFloat32Array()
	errors.resize(n_levels)
	var levels: Array = []
	for level in n_levels:
		var e := 0.0
		var idx_by_surface: Array = []
		for r: Dictionary in per_surface:
			e = maxf(e, float(r["errors"][level]))
			idx_by_surface.append(r["levels"][level])
		errors[level] = e
		levels.append(idx_by_surface)
	var desc := {"levels": levels, "errors": errors, "center": per_surface[0]["center"], "radius": float(per_surface[0]["radius"])}
	if path != "":
		write_cache_file(path, desc)
	desc["from_cache"] = false
	return desc


## Clé du cache : MD5 des positions et indices de chaque surface + paramètres.
static func cache_key(surfaces: Array, params: Dictionary) -> String:
	var ctx := HashingContext.new()
	ctx.start(HashingContext.HASH_MD5)
	ctx.update(("%d|%s|%s|%s|%d" % [CACHE_VERSION, params["ratio"], params["target_error"], params["weld_tolerance"], params["max_levels"]]).to_utf8_buffer())
	for arrays: Array in surfaces:
		ctx.update((arrays[Mesh.ARRAY_VERTEX] as PackedVector3Array).to_byte_array())
		var idx: Variant = arrays[Mesh.ARRAY_INDEX]
		if idx != null:
			ctx.update((idx as PackedInt32Array).to_byte_array())
		ctx.update(PackedByteArray([0xff]))
	return ctx.finish().hex_encode()


## Octets d'un descripteur (déterministes : relu puis réécrit = identique).
static func encode_cache(desc: Dictionary) -> PackedByteArray:
	var out := CACHE_MAGIC.to_ascii_buffer()
	var ver := PackedByteArray()
	ver.resize(4)
	ver.encode_u32(0, CACHE_VERSION)
	out.append_array(ver)
	out.append_array(var_to_bytes({"levels": desc["levels"], "errors": desc["errors"], "center": desc["center"], "radius": desc["radius"]}))
	return out


## Descripteur relu ; vide si absent, corrompu ou d'une autre version.
static func decode_cache(bytes: PackedByteArray) -> Dictionary:
	if bytes.size() < 8 or bytes.slice(0, 4).get_string_from_ascii() != CACHE_MAGIC or bytes.decode_u32(4) != CACHE_VERSION:
		return {}
	var v: Variant = bytes_to_var(bytes.slice(8))
	if typeof(v) != TYPE_DICTIONARY or not v.has_all(["levels", "errors", "center", "radius"]):
		return {}
	return v


static func read_cache_file(path: String) -> Dictionary:
	if not FileAccess.file_exists(path):
		return {}
	return decode_cache(FileAccess.get_file_as_bytes(path))


static func write_cache_file(path: String, desc: Dictionary) -> bool:
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(path.get_base_dir()))
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		push_warning("lodcells : cache inaccessible en écriture : " + path)
		return false
	f.store_buffer(encode_cache(desc))
	return true


## Chaîne d'erreurs du sélecteur pour un maillage à une échelle d'exemplaire
## donnée : les erreurs sont mesurées dans le repère du maillage, la caméra
## travaille dans celui de la scène — sans ce facteur, un GLB à l'échelle 0,1
## verrait ses erreurs surestimées 10× et garderait ses niveaux fins trop loin.
func _chain_for(lod: Dictionary, scale: float) -> int:
	var errors: PackedFloat32Array = lod["errors"]
	if errors.size() <= 1 or _selector == null:
		return -1
	var key := [lod["meshes"][0].get_rid(), snappedf(scale, 0.0001)]
	if _chain_cache.has(key):
		return _chain_cache[key]
	var scaled := PackedFloat32Array()
	scaled.resize(errors.size())
	for i in errors.size():
		scaled[i] = errors[i] * scale
	var chain: int = _selector.add_chain(scaled)
	_chain_cache[key] = chain
	return chain


func _process(_delta: float) -> void:
	if _selector == null or _cells.is_empty():
		return
	var cam := get_viewport().get_camera_3d()
	if cam == null:
		return
	_selector.hysteresis = hysteresis
	_selector.begin_frame(to_local(cam.global_position), cam.fov, get_viewport().get_visible_rect().size.y)
	var changed: PackedInt32Array = _selector.select_changed(threshold_px)
	for i in changed.size():
		var level := changed[i]
		if level < 0:
			continue
		var cell = _cells[i]
		if cell == null:
			continue
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


## Niveau courant de chaque cellule vivante (diagnostic / tests).
func levels() -> PackedInt32Array:
	var out := PackedInt32Array()
	for c in _cells:
		if c != null:
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
