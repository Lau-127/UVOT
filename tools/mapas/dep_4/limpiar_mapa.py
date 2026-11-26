#!/usr/bin/env python3
"""
Limpia un mapa de slam_toolbox / map_server quitando los "destellos" que deja
el lidar al atravesar cristales, y cierra el espacio libre con paredes.

Pasos:
  1. Dibuja los cortes manuales (segmentos en píxeles) como pared.
  2. Apertura morfológica del espacio libre (quita rayos finos).
  3. Se queda con las zonas libres de más de --area-min.
  4. Recupera el libre original pegado a lo que queda (radio de la apertura) y
     descarta lo que haya quedado al otro lado de un corte.
  5. Quita obstáculos sueltos lejos del espacio libre (lo visto tras el cristal).
  6. Rellena como libre los huecos desconocidos pequeños encerrados en libre.
  7. Cierra: todo libre que toca desconocido pasa a ocupado.

Uso:
  python3 limpiar_mapa.py dep_4.yaml --cortes cortes_dep_4.yaml -o dep_4_limpio
Genera dep_4_limpio.pgm, dep_4_limpio.yaml y dep_4_limpio_debug.png
(rojo = cierres añadidos, azul = cortes, amarillo = libre eliminado).
"""
import argparse, os
import numpy as np, cv2, yaml
from PIL import Image

FREE, UNK, OCC = 254, 205, 0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml_mapa")
    ap.add_argument("--cortes", help="YAML con lista 'cortes: [[x0,y0,x1,y1], ...]' en píxeles")
    ap.add_argument("-o", "--salida", required=True, help="nombre base de salida (sin extensión)")
    ap.add_argument("--radio", type=float, default=0.25, help="radio de apertura en m (def. 0.25)")
    ap.add_argument("--area-min", type=float, default=5.0, help="área mínima (m²) de zona libre a conservar")
    ap.add_argument("--dist-obst", type=float, default=0.30, help="distancia máx. (m) de un obstáculo al libre para conservarlo")
    ap.add_argument("--hueco-max", type=float, default=1.0, help="huecos desconocidos encerrados de menos de este área (m²) pasan a libre")
    ap.add_argument("--grosor-corte", type=int, default=3, help="grosor de los cortes en píxeles")
    a = ap.parse_args()

    with open(a.yaml_mapa) as f:
        meta = yaml.safe_load(f)
    res = float(meta["resolution"])
    img_path = os.path.join(os.path.dirname(os.path.abspath(a.yaml_mapa)), meta["image"])
    im = np.array(Image.open(img_path).convert("L"))
    if meta.get("negate", 0):
        im = 255 - im
    # Por si el mapa no viene en trinario puro
    p = (255 - im.astype(float)) / 255.0
    occ = (p > meta.get("occupied_thresh", 0.65)).astype(np.uint8)
    free = (p < meta.get("free_thresh", 0.196)).astype(np.uint8)

    cortes = np.zeros_like(occ)
    if a.cortes:
        with open(a.cortes) as f:
            for x0, y0, x1, y1 in yaml.safe_load(f)["cortes"]:
                cv2.line(cortes, (int(x0), int(y0)), (int(x1), int(y1)), 1, a.grosor_corte)
    free[cortes == 1] = 0

    r = max(1, int(round(a.radio / res)))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
    abierto = cv2.morphologyEx(free, cv2.MORPH_OPEN, k)

    n, lab, st, _ = cv2.connectedComponentsWithStats(abierto, connectivity=4)
    amin = a.area_min / res**2
    keep = np.isin(lab, [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= amin]).astype(np.uint8)

    # Recupera el libre original junto a lo conservado (bordes, rincones), sin cruzar cortes
    k2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 3, 2 * r + 3))
    keep = ((cv2.dilate(keep, k2) == 1) & (free == 1)).astype(np.uint8)
    # Se conserva la zona principal (la mayor) y cualquier otra zona grande que no toque un
    # corte: lo que queda al otro lado de un corte es lo visto a través del cristal.
    n, lab, st, _ = cv2.connectedComponentsWithStats(keep, connectivity=4)
    junto_corte = set(np.unique(lab[cv2.dilate(cortes, np.ones((5, 5), np.uint8)) == 1]))
    principal = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA])) if n > 1 else -1
    ids = [i for i in range(1, n) if i == principal or
           (st[i, cv2.CC_STAT_AREA] >= amin and i not in junto_corte)]
    keep = np.isin(lab, ids).astype(np.uint8)

    # Obstáculos: solo los cercanos al libre conservado, más los cortes
    d = max(1, int(round(a.dist_obst / res)))
    cerca = cv2.dilate(keep, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * d + 1, 2 * d + 1)))
    occ_out = ((occ == 1) & (cerca == 1)) | (cortes == 1)

    out = np.full(im.shape, UNK, np.uint8)
    out[keep == 1] = FREE
    out[occ_out] = OCC
    # Huecos desconocidos pequeños rodeados de libre (zonas no barridas) -> libre
    n, lab, st, _ = cv2.connectedComponentsWithStats((out == UNK).astype(np.uint8), connectivity=4)
    hmax = a.hueco_max / res**2
    for i in range(1, n):
        x, y, w, h, area = st[i]
        if area <= hmax and x > 0 and y > 0 and x + w < out.shape[1] and y + h < out.shape[0]:
            out[lab == i] = FREE
    # Cierre: libre que toca desconocido -> pared
    unk = (out == UNK).astype(np.uint8)
    borde = (cv2.dilate(unk, np.ones((3, 3), np.uint8)) == 1) & (out == FREE)
    out[borde] = OCC

    base = a.salida
    Image.fromarray(out).save(base + ".pgm")
    meta_out = dict(meta, image=os.path.basename(base) + ".pgm", mode="trinary", negate=0)
    with open(base + ".yaml", "w") as f:
        yaml.safe_dump(meta_out, f, sort_keys=False, default_flow_style=None)
    # Máscara de cierres para el mundo de Gazebo (paredes "artificiales")
    cierres = (borde | (cortes == 1)) & ~((occ == 1) & (cerca == 1))
    Image.fromarray((cierres * 255).astype(np.uint8)).save(base + "_cierres.png")

    vis = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    vis[(free == 1) & (keep == 0)] = (120, 230, 255)
    vis[borde] = (0, 0, 255)
    vis[cortes == 1] = (255, 80, 0)
    cv2.imwrite(base + "_debug.png", vis)
    print(f"Libre: {free.sum()} -> {keep.sum()} px; cierres: {int(cierres.sum())} px; "
          f"obstáculos: {occ.sum()} -> {int(occ_out.sum())} px")

if __name__ == "__main__":
    main()
