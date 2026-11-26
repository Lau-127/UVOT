#!/usr/bin/env python3
"""
Convierte un mapa de ocupación (yaml + pgm de map_server / slam_toolbox) en un
modelo estático de Gazebo (Harmonic, SDF 1.9) con las paredes extruidas, y un
mundo de prueba que lo incluye.

El marco del mundo coincide con el frame 'map': un punto (x, y) del mapa está en
(x, y) en Gazebo, así que la pose de spawn sirve también como pose inicial de AMCL.

Uso:
  python3 mapa_a_gazebo.py dep_4_limpio.yaml --cierres dep_4_limpio_cierres.png \
      --nombre dep_4 --salida gz --altura 1.0 --modo-cierres cristal

--modo-cierres (paredes añadidas para tapar los cristales):
  visible    pared igual que las demás (el lidar simulado la ve)
  cristal    pared azulada semitransparente (el gpu_lidar la ve igualmente)
  invisible  solo colisión, sin visual: el robot choca pero el gpu_lidar NO la ve,
             como pasa con el cristal real
"""
import argparse, os
import numpy as np, yaml
from PIL import Image


def rectangulos(mask):
    """Agrupa los píxeles a 1 en rectángulos (tramos por fila fusionados en vertical)."""
    H, W = mask.shape
    abiertos = {}  # (c0, c1) -> r0
    rects = []
    for r in range(H + 1):
        tramos = set()
        if r < H:
            fila = np.concatenate(([0], mask[r].astype(np.int8), [0]))
            d = np.diff(fila)
            ini, fin = np.where(d == 1)[0], np.where(d == -1)[0] - 1
            tramos = set(zip(ini.tolist(), fin.tolist()))
        for t in list(abiertos):
            if t not in tramos:
                rects.append((t[0], abiertos.pop(t), t[1], r - 1))
        for t in tramos:
            abiertos.setdefault(t, r)
    return rects  # (c0, r0, c1, r1) inclusivos


def escribir_obj(path, rects, H, res, ox, oy, altura):
    v, vn, f = [], [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1)], []
    for c0, r0, c1, r1 in rects:
        x0, x1 = ox + c0 * res, ox + (c1 + 1) * res
        y0, y1 = oy + (H - r1 - 1) * res, oy + (H - r0) * res
        b = len(v) + 1
        v += [(x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0),
              (x0, y0, altura), (x1, y0, altura), (x1, y1, altura), (x0, y1, altura)]
        caras = [((1, 2, 6, 5), 4), ((2, 3, 7, 6), 1), ((3, 4, 8, 7), 3),
                 ((4, 1, 5, 8), 2), ((5, 6, 7, 8), 5)]  # sin cara inferior
        for (a, bb, c, d), n in caras:
            f.append((b + a - 1, b + bb - 1, b + c - 1, n))
            f.append((b + a - 1, b + c - 1, b + d - 1, n))
    with open(path, "w") as fh:
        fh.write("# generado por mapa_a_gazebo.py\n")
        fh.writelines(f"v {x:.4f} {y:.4f} {z:.4f}\n" for x, y, z in v)
        fh.writelines(f"vn {a} {b} {c}\n" for a, b, c in vn)
        fh.writelines(f"f {a}//{n} {b}//{n} {c}//{n}\n" for a, b, c, n in f)
    return len(v), len(f)


MAT_PARED = """<material>
          <ambient>0.75 0.75 0.72 1</ambient><diffuse>0.85 0.85 0.82 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>"""
MAT_CRISTAL = """<material>
          <ambient>0.4 0.7 0.9 0.35</ambient><diffuse>0.4 0.7 0.9 0.35</diffuse>
          <specular>0.6 0.6 0.6 1</specular>
        </material>
        <transparency>0.6</transparency>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml_mapa")
    ap.add_argument("--cierres", help="PNG (máscara) de paredes añadidas que genera limpiar_mapa.py")
    ap.add_argument("--nombre", default="mapa")
    ap.add_argument("--salida", default="gz")
    ap.add_argument("--altura", type=float, default=1.0)
    ap.add_argument("--modo-cierres", choices=["visible", "cristal", "invisible"], default="cristal")
    ap.add_argument("--mundo", default="uvot_world", help="nombre del <world> (tus scripts usan uvot_world)")
    a = ap.parse_args()

    with open(a.yaml_mapa) as fh:
        meta = yaml.safe_load(fh)
    res = float(meta["resolution"]); ox, oy = meta["origin"][:2]
    if len(meta["origin"]) > 2 and abs(meta["origin"][2]) > 1e-9:
        raise SystemExit("Origen con yaw distinto de 0: no soportado")
    im = np.array(Image.open(os.path.join(os.path.dirname(os.path.abspath(a.yaml_mapa)), meta["image"])).convert("L"))
    if meta.get("negate", 0):
        im = 255 - im
    occ = (255 - im.astype(float)) / 255.0 > meta.get("occupied_thresh", 0.65)
    H = im.shape[0]

    cierres = np.zeros_like(occ)
    if a.cierres:
        cierres = (np.array(Image.open(a.cierres).convert("L")) > 127) & occ
    paredes = occ & ~cierres

    mdir = os.path.join(a.salida, "models", a.nombre)
    os.makedirs(os.path.join(mdir, "meshes"), exist_ok=True)
    rp = rectangulos(paredes)
    nv, nf = escribir_obj(os.path.join(mdir, "meshes", "paredes.obj"), rp, H, res, ox, oy, a.altura)
    print(f"paredes: {paredes.sum()} px -> {len(rp)} cajas, {nf} triángulos")
    bloques = f"""
      <collision name="paredes_col">
        <geometry><mesh><uri>model://{a.nombre}/meshes/paredes.obj</uri></mesh></geometry>
      </collision>
      <visual name="paredes_vis">
        <geometry><mesh><uri>model://{a.nombre}/meshes/paredes.obj</uri></mesh></geometry>
        {MAT_PARED}
      </visual>"""
    if cierres.any():
        rc = rectangulos(cierres)
        escribir_obj(os.path.join(mdir, "meshes", "cierres.obj"), rc, H, res, ox, oy, a.altura)
        print(f"cierres: {cierres.sum()} px -> {len(rc)} cajas (modo {a.modo_cierres})")
        bloques += f"""
      <collision name="cierres_col">
        <geometry><mesh><uri>model://{a.nombre}/meshes/cierres.obj</uri></mesh></geometry>
      </collision>"""
        if a.modo_cierres != "invisible":
            mat = MAT_PARED if a.modo_cierres == "visible" else MAT_CRISTAL
            bloques += f"""
      <visual name="cierres_vis">
        <geometry><mesh><uri>model://{a.nombre}/meshes/cierres.obj</uri></mesh></geometry>
        {mat}
      </visual>"""

    with open(os.path.join(mdir, "model.sdf"), "w") as fh:
        fh.write(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{a.nombre}">
    <static>true</static>
    <link name="link">{bloques}
    </link>
  </model>
</sdf>
""")
    with open(os.path.join(mdir, "model.config"), "w") as fh:
        fh.write(f"""<?xml version="1.0"?>
<model>
  <name>{a.nombre}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Paredes extruidas desde {meta['image']} (res {res} m, altura {a.altura} m)</description>
</model>
""")
    os.makedirs(os.path.join(a.salida, "worlds"), exist_ok=True)
    with open(os.path.join(a.salida, "worlds", f"{a.nombre}.sdf"), "w") as fh:
        fh.write(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{a.mundo}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>

    <include>
      <uri>model://{a.nombre}</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
""")
    print(f"Modelo: {mdir}\nMundo:  {os.path.join(a.salida, 'worlds', a.nombre + '.sdf')}")


if __name__ == "__main__":
    main()
