#!/usr/bin/env python3
"""Genera el YAML de Nav2 de UVOT para un robot y una fase de validación.

    python3 uvot_nav2_params.py <diff|skid|omni> <odom|firmware|amcl> [opciones]

Fases:
  odom      sin mapa, localización = odometría de ruedas pura (/<ns>/odom_wheels)
  firmware  sin mapa, localización = odometría del firmware (encoders + IMU, /<ns>/odom)
  amcl      con mapa: odometría del firmware + AMCL con el RPLIDAR S2

También se importa desde nav.launch.py (función generar).

Rellena todos los marcadores __X__ de nav2_params_common.yaml y escribe
nav2_params_<robot>_<fase>.yaml. Ese archivo se pasa tal cual a Nav2
(params_file); use_sim_time lo sigue poniendo el launch.
"""
import argparse
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
# Instalado: share/<paquete>/launch/ -> la plantilla está en share/<paquete>/config/
if not os.path.isfile(os.path.join(AQUI, "nav2_params_common.yaml")):
    AQUI = os.path.join(os.path.dirname(AQUI), "config")

# Huella: chasis 0.225 x 0.20 m; las ruedas sobresalen hasta |y| ~ 0.12 m.
# Rectángulo con ~1 cm de margen (en base_footprint).
FOOTPRINT = "[[0.125, 0.13], [0.125, -0.13], [-0.125, -0.13], [-0.125, 0.13]]"

ROBOTS = {
    # Límites del velocity_smoother: velocidades validadas en la batería
    # dinámica (0.6 m/s y 3 rad/s alcanzables) con margen.
    "diff": dict(amcl="nav2_amcl::DifferentialMotionModel", mppi="DiffDrive", min_y="0.5",
                 vmax="[0.5, 0.0, 1.9]", vmin="[-0.5, 0.0, -1.9]",
                 amax="[0.5, 0.0, 2.5]", amin="[-0.5, 0.0, -2.5]", critics=""),
    # El skid se trata como diferencial (el derrape lo absorbe B_SKID en el firmware)
    "skid": dict(amcl="nav2_amcl::DifferentialMotionModel", mppi="DiffDrive", min_y="0.5",
                 vmax="[0.5, 0.0, 1.9]", vmin="[-0.5, 0.0, -1.9]",
                 amax="[0.5, 0.0, 2.5]", amin="[-0.5, 0.0, -2.5]", critics=""),
    "omni": dict(amcl="nav2_amcl::OmniMotionModel", mppi="Omni", min_y="0.001",
                 vmax="[0.5, 0.5, 1.9]", vmin="[-0.5, -0.5, -1.9]",
                 amax="[0.5, 0.5, 2.5]", amin="[-0.5, -0.5, -2.5]",
                 critics=', "TwirlingCritic"'),
}

NAV_NODES = ["controller_server", "planner_server", "bt_navigator",
             "behavior_server", "global_costmap", "local_costmap"]


def generar(robot, fase, ns="robot1", mapa="", plantilla=None):
    """Devuelve el texto YAML de Nav2 para ese robot y esa fase."""
    r = ROBOTS[robot]
    ns = ns.strip("/")
    con_mapa = fase == "amcl"
    if con_mapa and not mapa:
        raise ValueError("La fase amcl necesita la ruta al .yaml del mapa")
    valores = {
        "__ROBOT_NS__": ns,
        "__AMCL_MOTION_MODEL__": r["amcl"],
        "__MPPI_MOTION_MODEL__": r["mppi"],
        "__MIN_Y_VEL__": r["min_y"],
        "__MPPI_EXTRA_CRITICS__": r["critics"],
        "__MAX_VELOCITY__": r["vmax"],
        "__MIN_VELOCITY__": r["vmin"],
        "__MAX_ACCEL__": r["amax"],
        "__MAX_DECEL__": r["amin"],
        "__FOOTPRINT__": FOOTPRINT,
        "__ODOM_TOPIC__": f"/{ns}/odom_wheels" if fase == "odom" else f"/{ns}/odom",
        "__GLOBAL_FRAME__": "map" if con_mapa else f"{ns}/odom",
        "__GLOBAL_ROLLING__": "false" if con_mapa else "true",
        "__GLOBAL_PLUGINS__": ('["static_layer", "obstacle_layer", "inflation_layer"]' if con_mapa
                               else '["obstacle_layer", "inflation_layer"]'),
        "__MAP_FILE__": os.path.expanduser(mapa),
        "__LIFECYCLE_NODES__": str((["map_server", "amcl"] if con_mapa else []) + NAV_NODES),
    }
    texto = open(plantilla or os.path.join(AQUI, "nav2_params_common.yaml")).read()
    for k, v in valores.items():
        texto = texto.replace(k, v)
    restos = sorted({p for p in texto.split() if p.startswith("__") and p.endswith("__")})
    if restos:
        raise ValueError(f"Quedan marcadores sin rellenar: {restos}")
    return f"# Generado con uvot_nav2_params.py {robot} {fase} --ns {ns}\n" + texto


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot", choices=ROBOTS)
    ap.add_argument("fase", choices=["odom", "firmware", "amcl"])
    ap.add_argument("--ns", default="robot1", help="namespace del robot (por defecto robot1)")
    ap.add_argument("--mapa", default="", help="ruta al .yaml del mapa (fase amcl)")
    ap.add_argument("--plantilla", default=None)
    ap.add_argument("--salida", default=None)
    a = ap.parse_args()
    try:
        texto = generar(a.robot, a.fase, a.ns, a.mapa, a.plantilla)
    except ValueError as e:
        sys.exit(str(e))
    salida = a.salida or f"nav2_params_{a.robot}_{a.fase}.yaml"
    with open(salida, "w") as f:
        f.write(texto)
    print(f"Escrito {salida}")


if __name__ == "__main__":
    main()
