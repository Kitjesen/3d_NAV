/**
 * Scene3D — Three.js 3D map visualization
 *
 * Coordinate mapping:
 *   World (LingTu): X right, Y forward, Z up
 *   Three.js:       X right, Y up,      Z toward camera
 *   → Three.js pos = (worldX, worldZ_height, -worldY)
 */
import { useRef, useEffect, forwardRef, useImperativeHandle } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import type { PathPoint, CostmapEvent, SlopeGridEvent, SceneGraphEvent } from '../types'

export interface Scene3DHandle {
  resetCamera(): void
}

interface Layers {
  grid:    boolean
  cloud:   boolean
  trail:   boolean
  path:    boolean
  goal:    boolean
  robot:   boolean
  costmap: boolean
  slope:   boolean
}

interface Scene3DProps {
  cloudFlat:    number[]
  savedMapFlat?: number[]
  costmap:      CostmapEvent | null
  slopeGrid:    SlopeGridEvent | null
  sceneGraph:   SceneGraphEvent | null
  robotX:       number
  robotY:       number
  yaw:          number
  trail:        Array<[number, number]>
  path:         PathPoint[]
  localPath:    PathPoint[]
  layers:       Layers
  pointSize:    number
  onPendingGoal: (x: number, y: number) => void
  pendingGoal?:  { x: number; y: number } | null
}

const Z_FLOOR   = -0.2  // include floor-level scan points
const Z_CEIL    = 2.8   // ignore points above ceiling (m)

// Turbo colormap (matches Foxglove/dimos default)
// Polynomial approximation of Google's turbo colormap
// Neutral cool→warm gradient for point cloud height coloring.
// Low (floor)   → cool teal (matches --accent)
// Mid           → neutral dim gray
// High (ceiling) → warm amber
// No rainbow — stays visually quiet so cost map / path are primary focus.
function turboColor(col: THREE.Color, t: number) {
  t = Math.max(0, Math.min(1, t))
  // Low teal (0.18, 0.55, 0.50) → mid gray (0.55, 0.55, 0.55) → high amber (0.78, 0.60, 0.35)
  const lerp = (a: number, b: number, x: number) => a + (b - a) * x
  let r: number, g: number, b: number
  if (t < 0.5) {
    const u = t * 2
    r = lerp(0.18, 0.55, u)
    g = lerp(0.55, 0.55, u)
    b = lerp(0.50, 0.55, u)
  } else {
    const u = (t - 0.5) * 2
    r = lerp(0.55, 0.78, u)
    g = lerp(0.55, 0.60, u)
    b = lerp(0.55, 0.35, u)
  }
  col.setRGB(r, g, b)
}

function removeFrom(scene: THREE.Scene, obj: THREE.Object3D | undefined | null) {
  if (!obj) return
  scene.remove(obj)
  if ((obj as THREE.Mesh).geometry) (obj as THREE.Mesh).geometry.dispose()
  const mat = (obj as THREE.Mesh).material
  if (mat) Array.isArray(mat) ? mat.forEach(m => m.dispose()) : mat.dispose()
}

export const Scene3D = forwardRef<Scene3DHandle, Scene3DProps>(function Scene3D(
  { cloudFlat, savedMapFlat, costmap, slopeGrid, sceneGraph, robotX, robotY, yaw, trail, path, localPath, layers, pointSize, onPendingGoal, pendingGoal },
  ref,
) {
  const mountRef   = useRef<HTMLDivElement>(null)
  const sceneRef   = useRef<THREE.Scene | null>(null)
  const cameraRef  = useRef<THREE.PerspectiveCamera | null>(null)
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null)
  const controlsRef = useRef<OrbitControls | null>(null)
  const rafRef     = useRef(0)

  // Scene objects — recreated on data change
  const voxelRef   = useRef<THREE.InstancedMesh | null>(null)
  const trailLineRef = useRef<THREE.Line | null>(null)
  const pathLineRef  = useRef<THREE.Line | null>(null)
  const localPathRef = useRef<THREE.Line | null>(null)
  const robotRef   = useRef<THREE.Group | null>(null)
  const goalRef        = useRef<THREE.Mesh | null>(null)
  const pendingGoalRef = useRef<THREE.Mesh | null>(null)
  const costmapMeshRef = useRef<THREE.Mesh | null>(null)
  const slopeMeshRef   = useRef<THREE.Mesh | null>(null)
  const gridRef        = useRef<THREE.GridHelper | null>(null)
  const floorRef   = useRef<THREE.Mesh | null>(null)
  const savedMapRef    = useRef<THREE.Points | null>(null)
  const sgGroupRef     = useRef<THREE.Group | null>(null)
  const raycaster  = useRef(new THREE.Raycaster())
  const robotPosRef = useRef({ x: 0, y: 0 })

  // ── Expose resetCamera ──────────────────────────────────────────
  useImperativeHandle(ref, () => ({
    resetCamera() {
      const { x, y } = robotPosRef.current
      cameraRef.current?.position.set(x, 20, -y + 18)
      if (controlsRef.current) {
        controlsRef.current.target.set(x, 0, -y)
        controlsRef.current.update()
      }
    },
  }), [])

  // ── Init (once) ─────────────────────────────────────────────────
  useEffect(() => {
    const mount = mountRef.current!
    const w = mount.clientWidth, h = mount.clientHeight

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x07070e)
    sceneRef.current = scene

    const camera = new THREE.PerspectiveCamera(50, w / h, 0.1, 500)
    camera.position.set(0, 20, 18)
    camera.lookAt(0, 0, 0)
    cameraRef.current = camera

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.setSize(w, h)
    mount.appendChild(renderer.domElement)
    rendererRef.current = renderer

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping  = true
    controls.dampingFactor  = 0.1
    controls.minDistance    = 2
    controls.maxDistance    = 120
    controls.maxPolarAngle  = Math.PI * 0.475
    controlsRef.current = controls

    scene.add(new THREE.AmbientLight(0xffffff, 0.55))
    const dir = new THREE.DirectionalLight(0x7799ff, 0.9)
    dir.position.set(10, 25, 8)
    scene.add(dir)

    const grid = new THREE.GridHelper(300, 150, 0x18182e, 0x111126)
    grid.position.y = -0.02
    scene.add(grid)
    gridRef.current = grid

    // Invisible floor plane for click raycasting
    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(600, 600),
      new THREE.MeshBasicMaterial({ visible: false, side: THREE.DoubleSide }),
    )
    floor.rotation.x = -Math.PI / 2
    scene.add(floor)
    floorRef.current = floor

    const animate = () => {
      rafRef.current = requestAnimationFrame(animate)
      controls.update()
      renderer.render(scene, camera)
    }
    animate()

    const ro = new ResizeObserver(() => {
      const nw = mount.clientWidth, nh = mount.clientHeight
      camera.aspect = nw / nh
      camera.updateProjectionMatrix()
      renderer.setSize(nw, nh)
    })
    ro.observe(mount)

    return () => {
      cancelAnimationFrame(rafRef.current)
      ro.disconnect()
      controls.dispose()
      renderer.dispose()
      if (mount.contains(renderer.domElement)) mount.removeChild(renderer.domElement)
    }
  }, [])

  // ── Grid visibility ─────────────────────────────────────────────
  useEffect(() => {
    if (gridRef.current) gridRef.current.visible = layers.grid
  }, [layers.grid])

  // ── Voxel height map ────────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return

    if (voxelRef.current) {
      scene.remove(voxelRef.current)
      voxelRef.current.geometry.dispose()
      ;(voxelRef.current.material as THREE.Material).dispose()
      voxelRef.current = null
    }

    if (!layers.cloud || cloudFlat.length < 3) return

    // Collect all valid points (skip floor/ceiling noise)
    const positions: number[] = []
    const colors:    number[] = []

    // First pass: collect Z values for percentile range
    const allZ: number[] = []
    for (let i = 2; i < cloudFlat.length; i += 3) {
      const wz = cloudFlat[i]
      if (wz > Z_FLOOR && wz < Z_CEIL) allZ.push(wz)
    }
    if (allZ.length === 0) return
    allZ.sort((a, b) => a - b)
    const zLo = allZ[Math.floor(allZ.length * 0.02)]
    const zHi = allZ[Math.floor(allZ.length * 0.98)]
    const zSpan = Math.max(zHi - zLo, 0.1)

    // Second pass: build point positions + per-point colors
    const col = new THREE.Color()
    for (let i = 0; i + 2 < cloudFlat.length; i += 3) {
      const wx = cloudFlat[i], wy = cloudFlat[i + 1], wz = cloudFlat[i + 2]
      if (wz < Z_FLOOR || wz > Z_CEIL) continue
      positions.push(wx, wz, -wy)   // Three.js: Y=height, Z=-worldY
      const t = Math.max(0, Math.min(1, (wz - zLo) / zSpan))
      turboColor(col, t)
      colors.push(col.r, col.g, col.b)
    }
    if (positions.length === 0) return

    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3))
    geo.setAttribute('color',    new THREE.Float32BufferAttribute(colors,    3))

    const mat  = new THREE.PointsMaterial({ size: pointSize, vertexColors: true, sizeAttenuation: true })
    const mesh = new THREE.Points(geo, mat)

    scene.add(mesh)
    voxelRef.current = mesh as unknown as THREE.InstancedMesh
  }, [cloudFlat, layers.cloud])

  // ── Live point size update ─────────────────────────────────────
  useEffect(() => {
    const mesh = voxelRef.current
    if (!mesh) return
    const mat = (mesh as unknown as THREE.Points).material as THREE.PointsMaterial
    if (mat?.isPointsMaterial) { mat.size = pointSize; mat.needsUpdate = true }
  }, [pointSize])

  // ── Trail ───────────────────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (trailLineRef.current) { removeFrom(scene, trailLineRef.current); trailLineRef.current = null }
    if (!layers.trail || trail.length < 2) return

    const pts = trail.map(([x, y]) => new THREE.Vector3(x, 0.06, -y))
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: 0x5eead4, transparent: true, opacity: 0.75 }),
    )
    scene.add(line)
    trailLineRef.current = line
  }, [trail, layers.trail])

  // ── Path ────────────────────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (pathLineRef.current) { removeFrom(scene, pathLineRef.current); pathLineRef.current = null }
    if (!layers.path || path.length < 2) return

    const rawPts = path.map(p => new THREE.Vector3(p.x, 0.1, -p.y))
    // Catmull-Rom spline for smooth path display
    const curve = new THREE.CatmullRomCurve3(rawPts, false, 'catmullrom', 0.5)
    const pts = curve.getPoints(Math.max(rawPts.length * 6, 60))
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: 0xa855f7 }),
    )
    scene.add(line)
    pathLineRef.current = line
  }, [path, layers.path])

  // ── Local path (amber, thinner — obstacle-avoidance path from LocalPlanner)
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (localPathRef.current) { removeFrom(scene, localPathRef.current); localPathRef.current = null }
    if (!layers.path || localPath.length < 2) return

    const pts = localPath.map(p => new THREE.Vector3(p.x, 0.12, -p.y))
    const geo = new THREE.BufferGeometry().setFromPoints(pts)
    const mat = new THREE.LineBasicMaterial({ color: 0xf59e0b, linewidth: 1 })  // amber
    const line = new THREE.Line(geo, mat)
    scene.add(line)
    localPathRef.current = line
  }, [localPath, layers.path])

  // ── Robot model ─────────────────────────────────────────────────
  useEffect(() => {
    robotPosRef.current = { x: robotX, y: robotY }
    const scene = sceneRef.current
    if (!scene) return

    if (!robotRef.current) {
      const g = new THREE.Group()

      // Body wireframe box
      g.add(new THREE.LineSegments(
        new THREE.EdgesGeometry(new THREE.BoxGeometry(0.68, 0.22, 0.34)),
        new THREE.LineBasicMaterial({ color: 0xf472b6 }),
      ))

      // Heading arrow
      g.add(new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0.3, 0.11, 0),
        0.55, 0xffffff, 0.18, 0.1,
      ))

      // Four leg dots (foot contact points)
      const legGeo = new THREE.SphereGeometry(0.04, 5, 5)
      const legMat = new THREE.MeshBasicMaterial({ color: 0xe2e8f0 });
      ([
        [0.28, -0.11,  0.18],
        [-0.28, -0.11, 0.18],
        [0.28, -0.11, -0.18],
        [-0.28, -0.11, -0.18],
      ] as [number, number, number][]).forEach(([x, y, z]) => {
        const m = new THREE.Mesh(legGeo, legMat)
        m.position.set(x, y, z)
        g.add(m)
      })

      // Camera frustum (sky-blue wireframe pyramid)
      const fp = [
        new THREE.Vector3(0.35, 0.11, 0),     // apex
        new THREE.Vector3(0.9,  0.48, 0.38),  // TL
        new THREE.Vector3(0.9,  0.48, -0.38), // TR
        new THREE.Vector3(0.9, -0.08, 0.38),  // BL
        new THREE.Vector3(0.9, -0.08, -0.38), // BR
      ]
      const edges = [[0,1],[0,2],[0,3],[0,4],[1,2],[2,4],[4,3],[3,1]]
      const fPos: number[] = []
      edges.forEach(([a, b]) => {
        fPos.push(fp[a].x, fp[a].y, fp[a].z, fp[b].x, fp[b].y, fp[b].z)
      })
      const fGeo = new THREE.BufferGeometry()
      fGeo.setAttribute('position', new THREE.Float32BufferAttribute(fPos, 3))
      g.add(new THREE.LineSegments(fGeo, new THREE.LineBasicMaterial({
        color: 0x38bdf8, transparent: true, opacity: 0.55,
      })))

      scene.add(g)
      robotRef.current = g
    }

    robotRef.current.visible = layers.robot
    if (layers.robot) {
      robotRef.current.position.set(robotX, 0, -robotY)
      robotRef.current.rotation.y = yaw
    }
  }, [robotX, robotY, yaw, layers.robot])

  // ── Goal marker ─────────────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (goalRef.current) { removeFrom(scene, goalRef.current); goalRef.current = null }
    if (!layers.goal || path.length === 0) return

    const last = path[path.length - 1]
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.3, 8, 8),
      new THREE.MeshBasicMaterial({ color: 0xfbbf24, wireframe: true }),
    )
    mesh.position.set(last.x, 0.3, -last.y)
    scene.add(mesh)
    goalRef.current = mesh
  }, [path, layers.goal])

  // ── Pending goal marker ────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (pendingGoalRef.current) { removeFrom(scene, pendingGoalRef.current); pendingGoalRef.current = null }
    if (!pendingGoal) return

    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.4, 0.06, 8, 32),
      new THREE.MeshBasicMaterial({ color: 0x06b6d4 }),
    )
    ring.rotation.x = Math.PI / 2
    ring.position.set(pendingGoal.x, 0.05, -pendingGoal.y)
    scene.add(ring)
    pendingGoalRef.current = ring
  }, [pendingGoal])

  // ── Costmap overlay ────────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return

    if (costmapMeshRef.current) {
      scene.remove(costmapMeshRef.current)
      costmapMeshRef.current.geometry.dispose()
      ;(costmapMeshRef.current.material as THREE.MeshBasicMaterial).map?.dispose()
      ;(costmapMeshRef.current.material as THREE.Material).dispose()
      costmapMeshRef.current = null
    }

    if (!costmap || !layers.costmap) return

    const { grid_b64, cols, resolution, origin } = costmap
    const bytes = Uint8Array.from(atob(grid_b64), c => c.charCodeAt(0))
    const rows  = Math.round(bytes.length / cols)
    if (rows <= 0 || cols <= 0) return

    // Draw costmap to an offscreen canvas
    const canvas = document.createElement('canvas')
    canvas.width  = cols
    canvas.height = rows
    const ctx = canvas.getContext('2d')!
    const img = ctx.createImageData(cols, rows)

    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const val  = bytes[r * cols + c]
        const o    = (r * cols + c) * 4
        // Unified cool→warm single-axis gradient (no rainbow).
        //   0         transparent (free space)
        //   1-40      accent dim (safe fringe, teal tint, low alpha)
        //   40-80     neutral warm (rising cost)
        //   80-100    red (impassable)
        if (val === 0) {
          img.data[o] = img.data[o+1] = img.data[o+2] = img.data[o+3] = 0
        } else if (val >= 80) {
          // LETHAL / high cost → muted red
          const t = Math.min(1, (val - 80) / 20)
          img.data[o]     = 200
          img.data[o + 1] = Math.round(60 - 20 * t)
          img.data[o + 2] = Math.round(60 - 20 * t)
          img.data[o + 3] = Math.round(110 + 20 * t)
        } else if (val >= 40) {
          // mid cost → warm neutral (desaturated amber)
          const t = (val - 40) / 40
          img.data[o]     = Math.round(140 + 60 * t)
          img.data[o + 1] = Math.round(100 - 40 * t)
          img.data[o + 2] = Math.round(80 - 20 * t)
          img.data[o + 3] = Math.round(70 + 40 * t)
        } else {
          // low cost → teal (matches --accent), very subtle
          const t = val / 40
          img.data[o]     = Math.round(40 + 20 * t)
          img.data[o + 1] = Math.round(140 + 40 * t)
          img.data[o + 2] = Math.round(130 + 30 * t)
          img.data[o + 3] = Math.round(20 + 40 * t)
        }
      }
    }
    ctx.putImageData(img, 0, 0)

    const tex = new THREE.CanvasTexture(canvas)
    tex.flipY = false  // grid[iy,ix] row0=Y_min — no flip needed
    tex.minFilter = THREE.LinearFilter
    tex.magFilter = THREE.NearestFilter

    const sizeX = cols * resolution
    const sizeY = rows * resolution
    const geo   = new THREE.PlaneGeometry(sizeX, sizeY)
    const mat   = new THREE.MeshBasicMaterial({
      map: tex, transparent: true, depthWrite: false,
    })
    const mesh = new THREE.Mesh(geo, mat)
    mesh.rotation.x = -Math.PI / 2
    mesh.position.set(
      origin[0] + sizeX / 2,
      0.01,
      -(origin[1] + sizeY / 2),
    )
    scene.add(mesh)
    costmapMeshRef.current = mesh
  }, [costmap, layers.costmap])

  // ── Costmap visibility toggle ──────────────────────────────────
  useEffect(() => {
    if (costmapMeshRef.current) costmapMeshRef.current.visible = layers.costmap
  }, [layers.costmap])

  // ── Slope grid overlay (green→yellow→red) ──────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return

    if (slopeMeshRef.current) {
      scene.remove(slopeMeshRef.current)
      slopeMeshRef.current.geometry.dispose()
      ;(slopeMeshRef.current.material as THREE.MeshBasicMaterial).map?.dispose()
      ;(slopeMeshRef.current.material as THREE.Material).dispose()
      slopeMeshRef.current = null
    }

    if (!slopeGrid || !layers.slope) return

    const { grid_b64, cols, resolution, origin } = slopeGrid
    const bytes = Uint8Array.from(atob(grid_b64), c => c.charCodeAt(0))
    const rows = Math.round(bytes.length / cols)
    if (rows <= 0 || cols <= 0) return

    const canvas = document.createElement('canvas')
    canvas.width = cols
    canvas.height = rows
    const ctx = canvas.getContext('2d')!
    const img = ctx.createImageData(cols, rows)

    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const raw = bytes[r * cols + c]  // no manual flip — CanvasTexture.flipY handles it
        const deg = raw * (90.0 / 255.0)
        const o = (r * cols + c) * 4
        if (deg < 3) {
          // flat — fully transparent
          img.data[o] = img.data[o+1] = img.data[o+2] = img.data[o+3] = 0
        } else if (deg < 15) {
          // mild slope — green
          const t = (deg - 3) / 12
          img.data[o]     = Math.round(40 * t)
          img.data[o + 1] = Math.round(180 + 40 * t)
          img.data[o + 2] = Math.round(60 * t)
          img.data[o + 3] = Math.round(30 + 50 * t)
        } else if (deg < 25) {
          // moderate slope — yellow
          const t = (deg - 15) / 10
          img.data[o]     = Math.round(200 + 55 * t)
          img.data[o + 1] = Math.round(200 - 60 * t)
          img.data[o + 2] = 30
          img.data[o + 3] = Math.round(80 + 40 * t)
        } else {
          // steep slope — red
          img.data[o]     = 240
          img.data[o + 1] = 50
          img.data[o + 2] = 50
          img.data[o + 3] = 140
        }
      }
    }
    ctx.putImageData(img, 0, 0)

    const tex = new THREE.CanvasTexture(canvas)
    tex.flipY = false  // grid[iy,ix] row0=Y_min — no flip needed
    tex.minFilter = THREE.LinearFilter
    tex.magFilter = THREE.NearestFilter

    const sizeX = cols * resolution
    const sizeY = rows * resolution
    const geo = new THREE.PlaneGeometry(sizeX, sizeY)
    const mat = new THREE.MeshBasicMaterial({
      map: tex, transparent: true, depthWrite: false,
    })
    const mesh = new THREE.Mesh(geo, mat)
    mesh.rotation.x = -Math.PI / 2
    mesh.position.set(
      origin[0] + sizeX / 2,
      0.02,
      -(origin[1] + sizeY / 2),
    )
    scene.add(mesh)
    slopeMeshRef.current = mesh
  }, [slopeGrid, layers.slope])

  // ── Saved map cloud (gray, background layer) ───────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (savedMapRef.current) {
      scene.remove(savedMapRef.current)
      savedMapRef.current.geometry.dispose()
      ;(savedMapRef.current.material as THREE.Material).dispose()
      savedMapRef.current = null
    }
    if (!savedMapFlat || savedMapFlat.length < 3) return
    const positions: number[] = []
    for (let i = 0; i + 2 < savedMapFlat.length; i += 3) {
      const wx = savedMapFlat[i], wy = savedMapFlat[i + 1], wz = savedMapFlat[i + 2]
      if (wz < Z_FLOOR || wz > Z_CEIL) continue
      positions.push(wx, wz, -wy)
    }
    if (positions.length === 0) return
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3))
    const mat = new THREE.PointsMaterial({ size: 0.06, color: 0x5566aa, sizeAttenuation: true, opacity: 0.7, transparent: true })
    const pts = new THREE.Points(geo, mat)
    scene.add(pts)
    savedMapRef.current = pts
  }, [savedMapFlat])

  // ── Semantic scene graph (objects + labels) ────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (sgGroupRef.current) {
      sgGroupRef.current.traverse(obj => {
        if ((obj as THREE.Mesh).geometry) (obj as THREE.Mesh).geometry.dispose()
        const mat = (obj as THREE.Mesh | THREE.Sprite).material as THREE.Material | THREE.Material[]
        if (mat) Array.isArray(mat) ? mat.forEach(m => m.dispose()) : mat.dispose()
      })
      scene.remove(sgGroupRef.current)
      sgGroupRef.current = null
    }
    if (!sceneGraph?.objects?.length) return
    const group = new THREE.Group()
    for (const obj of sceneGraph.objects) {
      const tx = obj.x, tz = -obj.y, ty = 0.5
      // Sphere marker
      const sphereGeo = new THREE.SphereGeometry(0.2, 8, 8)
      const conf = Math.max(0, Math.min(1, obj.confidence ?? 0.5))
      const sphereMat = new THREE.MeshBasicMaterial({ color: new THREE.Color().setHSL(conf * 0.33, 1, 0.55) })
      const sphere = new THREE.Mesh(sphereGeo, sphereMat)
      sphere.position.set(tx, ty, tz)
      group.add(sphere)
      // Ground line
      const lineGeo = new THREE.BufferGeometry()
      lineGeo.setAttribute('position', new THREE.Float32BufferAttribute([tx, 0, tz, tx, ty, tz], 3))
      group.add(new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0x44ffaa, opacity: 0.5, transparent: true })))
      // Text label sprite
      const cv = document.createElement('canvas')
      cv.width = 256; cv.height = 64
      const ctx = cv.getContext('2d')!
      ctx.fillStyle = 'rgba(8,10,24,0.82)'
      ctx.beginPath()
      ;(ctx as CanvasRenderingContext2D & { roundRect: (...args: unknown[]) => void }).roundRect(2, 2, 252, 60, 8)
      ctx.fill()
      ctx.font = 'bold 26px sans-serif'
      ctx.fillStyle = '#a5f3fc'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'middle'
      ctx.fillText(obj.label, 128, 32)
      const tex = new THREE.CanvasTexture(cv)
      const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true }))
      sprite.position.set(tx, ty + 0.9, tz)
      sprite.scale.set(2.2, 0.55, 1)
      group.add(sprite)
    }
    scene.add(group)
    sgGroupRef.current = group
  }, [sceneGraph])

  // ── Click vs drag detection ────────────────────────────────────
  const mouseDownPos = useRef<{ x: number; y: number } | null>(null)

  const handleMouseDown = (e: React.MouseEvent<HTMLDivElement>) => {
    mouseDownPos.current = { x: e.clientX, y: e.clientY }
  }

  const handleMouseUp = (e: React.MouseEvent<HTMLDivElement>) => {
    const down = mouseDownPos.current
    if (!down) return
    const dx = e.clientX - down.x
    const dy = e.clientY - down.y
    // Only treat as click if mouse moved < 5px (not a drag/orbit)
    if (Math.hypot(dx, dy) >= 5) return

    const renderer = rendererRef.current
    const camera   = cameraRef.current
    const floor    = floorRef.current
    if (!renderer || !camera || !floor) return

    const rect = renderer.domElement.getBoundingClientRect()
    const ndc  = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width)  * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    )
    raycaster.current.setFromCamera(ndc, camera)
    const hits = raycaster.current.intersectObject(floor)
    if (hits.length > 0) {
      const p = hits[0].point
      onPendingGoal(p.x, -p.z)  // convert Three.js back to world coords
    }
  }

  return (
    <div
      ref={mountRef}
      onMouseDown={handleMouseDown}
      onMouseUp={handleMouseUp}
      style={{ width: '100%', height: '100%', cursor: 'crosshair' }}
    />
  )
})
