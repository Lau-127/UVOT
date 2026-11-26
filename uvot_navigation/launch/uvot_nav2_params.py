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
# Rectángulo con ~1 cm de margen, en base_footprint (centro de giro).
# En el diff base_footprint está en el eje, 0.0825 m por detrás del centro del
# chasis (structure.xacro, base_x): la huella va desplazada hacia delante.
def _huella(dx=0.0):
    x1, x0 = 0.125 + dx, -0.125 + dx
    return (f"[[{x1:g}, 0.13], [{x1:g}, -0.13], "
            f"[{x0:g}, -0.13], [{x0:g}, 0.13]]")


DIFF_BASE_X = 0.0825   # = base_x del diff en robot_core.xacro

# Critics del MPPI que dependen de la cinemática (se añaden a la lista común):
#  - diff/skid: PathAngleCritic (encarar la ruta) y PreferForwardCritic (preferir
#    avanzar a retroceder);
#  - omni: TwirlingCritic. Sin PathAngle ni PreferForward, que obligaban al omni
#    a encarar la ruta y penalizaban moverse hacia atrás o de lado: la vuelta
#    tardaba ~25 s frente a ~16 s en diff/skid.
DIFF_CRITICS = ', "PathAngleCritic", "PreferForwardCritic"'

# shim: True envuelve el MPPI en el RotationShimController.
# shim_ang: desvío ruta-rumbo (rad) a partir del cual el shim gira en el sitio al
#   empezar; 3.2 (> pi) en el omni = nunca, solo hace el giro final.
# Límites de cada robot: v = (vx, vy, wz) en m/s y rad/s, a = (ax, ay, az) en
# m/s² y rad/s². Se usan IGUALES en el velocity_smoother y en el MPPI, para que
# el controlador no planifique movimientos que luego el smoother recorta.
# Valores de partida para el ajuste del MPPI (lo validado en la batería
# dinámica era 0.6 m/s y 3 rad/s alcanzables): 0.3 m/s y 1.0 rad/s dejan margen
# de PWM y reducen el derrape del skid. Con la batería, el antiguo era
# v = (0.5, 0/0.5, 1.9), a = (0.5, 0/0.5, 2.5).
ROBOTS = {
    "diff": dict(amcl="nav2_amcl::DifferentialMotionModel", mppi="DiffDrive", min_y="0.5",
                 v=(0.3, 0.0, 1.0), a=(0.5, 0.0, 2.5), critics=DIFF_CRITICS, shim=True, shim_ang=0.785,
                 huella=_huella(DIFF_BASE_X)),
    # El skid se trata como diferencial (el derrape lo absorbe B_SKID en el firmware)
    "skid": dict(amcl="nav2_amcl::DifferentialMotionModel", mppi="DiffDrive", min_y="0.5",
                 v=(0.3, 0.0, 1.0), a=(0.5, 0.0, 2.5), critics=DIFF_CRITICS, shim=True, shim_ang=0.785,
                 huella=_huella()),
    "omni": dict(amcl="nav2_amcl::OmniMotionModel", mppi="Omni", min_y="0.001",
                 v=(0.3, 0.3, 1.0), a=(0.5, 0.5, 2.5),
                 critics=', "TwirlingCritic"', shim=True, shim_ang=3.2,
                 huella=_huella()),
}

GOAL_CHECKERS = ("general", "precise")


def _num(x):
    """Siempre como double ('1.0', no '1'): ROS 2 no admite enteros en un
    parámetro double ni listas que mezclen enteros y decimales."""
    s = f"{float(x) + 0.0:g}"  # + 0.0 convierte -0.0 en 0.0
    return s if any(c in s for c in ".en") else s + ".0"


def _lista(t):
    return "[" + ", ".join(_num(x) for x in t) + "]"


NAV_NODES = ["controller_server", "planner_server", "bt_navigator",
             "behavior_server", "global_costmap", "local_costmap"]


def _aplicar_ajustes(texto, ajustes):
    """Sobrescribe parámetros de controller_server.FollowPath.

    ajustes: 'Critico.param=valor,param=valor' (rutas relativas a FollowPath),
    p. ej. 'TwirlingCritic.twirling_cost_weight=40,vx_std=0.3'. Cada valor toma
    el tipo del que ya había (double sigue siendo double aunque se escriba 40).
    """
    import yaml
    d = yaml.safe_load(texto)
    fp = d["controller_server"]["ros__parameters"]["FollowPath"]
    for par in filter(None, (p.strip() for p in ajustes.split(","))):
        clave, _, valor = par.partition("=")
        if not valor:
            raise ValueError(f"Ajuste sin valor: '{par}' (formato clave=valor)")
        *ruta, hoja = clave.strip().split(".")
        nodo = fp
        for k in ruta:
            if k not in nodo:
                raise ValueError(f"'{k}' no existe en FollowPath ({clave})")
            nodo = nodo[k]
        if hoja not in nodo:
            raise ValueError(f"'{clave}' no existe en FollowPath")
        v = yaml.safe_load(valor)
        antes = nodo[hoja]
        if isinstance(antes, float) and isinstance(v, int) and not isinstance(v, bool):
            v = float(v)
        elif type(antes) is not type(v):
            raise ValueError(f"'{clave}': era {type(antes).__name__} ({antes}), "
                             f"se ha dado {type(v).__name__} ({valor})")
        nodo[hoja] = v
    # Mapas en bloque y listas en línea, como en la plantilla (el parser de
    # parámetros de ROS 2 no admite mapas en línea {a: b})
    class _Dumper(yaml.SafeDumper):
        pass
    _Dumper.add_representer(list, lambda dm, v: dm.represent_sequence(
        "tag:yaml.org,2002:seq", v, flow_style=True))
    return yaml.dump(d, Dumper=_Dumper, sort_keys=False, default_flow_style=False,
                     allow_unicode=True, width=1000)


def generar(robot, fase, ns="robot1", mapa="", plantilla=None, goal_checker="general",
            xy_tol=0.05, yaw_tol=0.10, ajustes=""):
    """Devuelve el texto YAML de Nav2 para ese robot y esa fase.

    goal_checker: 'general' (0.35 m / 0.5 rad) o 'precise' (xy_tol m / yaw_tol
    rad; por defecto 0.05 / 0.10), para los ensayos de validación y el ajuste.
    ajustes: sobrescrituras de FollowPath (ver _aplicar_ajustes), para probar
    parámetros del controlador sin tocar la plantilla.
    """
    r = ROBOTS[robot]
    if goal_checker not in GOAL_CHECKERS:
        raise ValueError(f"goal_checker '{goal_checker}' no válido: {GOAL_CHECKERS}")
    (vx, vy, wz), (ax, ay, az) = r["v"], r["a"]
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
        "__MAX_VELOCITY__": _lista((vx, vy, wz)),
        "__MIN_VELOCITY__": _lista((-vx, -vy, -wz)),
        "__MAX_ACCEL__": _lista((ax, ay, az)),
        "__MAX_DECEL__": _lista((-ax, -ay, -az)),
        # MPPI: mismos límites que el velocity_smoother. En diff y skid el
        # modelo DiffDrive no usa la componente lateral; se deja vy/ay en 0.5
        # (como estaba) en vez de 0 para no dar valores nulos al MPPI.
        "__VX_MAX__": _num(vx),
        # Marcha atrás permitida (validada en la batería): sin ella, un robot
        # diferencial que llega orientado pero con el objetivo detrás no puede
        # corregir y se bloquea. PreferForwardCritic sigue priorizando avanzar.
        "__VX_MIN__": _num(-vx),
        "__VY_MAX__": _num(vy if vy > 0 else 0.5),
        "__WZ_MAX__": _num(wz),
        "__AX_MAX__": _num(ax),
        "__AX_MIN__": _num(-ax),
        "__AY_MAX__": _num(ay if ay > 0 else 0.5),
        "__AZ_MAX__": _num(az),
        "__GOAL_CHECKER__": f"{goal_checker}_goal_checker",
        "__XY_TOL__": _num(xy_tol),
        "__YAW_TOL__": _num(yaw_tol),
        # El MPPI corrige muy despacio unos pocos grados de orientación (ruido de
        # muestreo frente a una corrección pequeña): el giro final (y en diff/skid
        # también el inicial) lo hace el RotationShimController.
        "__SHIM_ANG__": _num(r["shim_ang"]),
        "__FOLLOWPATH_PLUGIN__": ("nav2_rotation_shim_controller::RotationShimController"
                                  if r["shim"] else "nav2_mppi_controller::MPPIController"),
        "__FOOTPRINT__": r["huella"],
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
    if ajustes:
        texto = _aplicar_ajustes(texto, ajustes)
    return (f"# Generado con uvot_nav2_params.py {robot} {fase} --ns {ns}"
            f" --goal-checker {goal_checker} --xy-tol {xy_tol} --yaw-tol {yaw_tol}"
            f" --ajustes '{ajustes}'\n" + texto)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot", choices=ROBOTS)
    ap.add_argument("fase", choices=["odom", "firmware", "amcl"])
    ap.add_argument("--ns", default="robot1", help="namespace del robot (por defecto robot1)")
    ap.add_argument("--mapa", default="", help="ruta al .yaml del mapa (fase amcl)")
    ap.add_argument("--goal-checker", default="general", choices=GOAL_CHECKERS,
                    help="general (0.35 m / 0.5 rad) o precise (0.05 m / 0.1 rad)")
    ap.add_argument("--xy-tol", type=float, default=0.05,
                    help="tolerancia xy del precise_goal_checker (m)")
    ap.add_argument("--yaw-tol", type=float, default=0.10,
                    help="tolerancia de orientación del precise_goal_checker (rad)")
    ap.add_argument("--ajustes", default="",
                    help="sobrescribe FollowPath: 'TwirlingCritic.twirling_cost_weight=40,vx_std=0.3'")
    ap.add_argument("--plantilla", default=None)
    ap.add_argument("--salida", default=None)
    a = ap.parse_args()
    try:
        texto = generar(a.robot, a.fase, a.ns, a.mapa, a.plantilla, a.goal_checker,
                        a.xy_tol, a.yaw_tol, a.ajustes)
    except ValueError as e:
        sys.exit(str(e))
    salida = a.salida or f"nav2_params_{a.robot}_{a.fase}_{a.goal_checker}.yaml"
    with open(salida, "w") as f:
        f.write(texto)
    print(f"Escrito {salida}")


if __name__ == "__main__":
    main()