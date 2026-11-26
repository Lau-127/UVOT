# dep_4: mapa limpio y mundo de Gazebo

## Contenido
- `dep_4_limpio.pgm/.yaml`: mapa sin destellos. Mismo origen y resolución que el original, así que vale para Nav2 sin más cambios.
- `dep_4_limpio_debug.png`: rojo = paredes de cierre añadidas, azul = cortes manuales, amarillo = espacio libre eliminado.
- `gz/models/dep_4/`: modelo estático con las paredes (`paredes.obj`) y los cierres de los cristales (`cierres.obj`).
- `gz/worlds/dep_4.sdf`: mundo de prueba con `<world name="uvot_world">`, para que tus scripts (`GZ_WORLD`) funcionen igual.
- `limpiar_mapa.py`, `cortes_dep_4.yaml`, `mapa_a_gazebo.py`: para regenerar todo.

## Probarlo
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/gz/models
gz sim -r gz/worlds/dep_4.sdf
```
El marco del mundo coincide con el frame `map`: el punto (0, 0) del mapa está en el pasillo, y la pose de spawn vale también como `initial_pose` de AMCL.

## Integrarlo en uvot_gazebo
1. Copia `gz/models/dep_4` a `uvot_gazebo/models/` y añade `install(DIRECTORY models ...)` en el CMakeLists si aún no lo tienes.
2. En el launch, antes de lanzar `gz_sim`:
   ```python
   AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
       os.path.join(get_package_share_directory('uvot_gazebo'), 'models'))
   ```
3. En tu mundo actual basta con añadir:
   ```xml
   <include><uri>model://dep_4</uri></include>
   ```

## Regenerar
```bash
python3 limpiar_mapa.py dep_4.yaml --cortes cortes_dep_4.yaml -o dep_4_limpio
python3 mapa_a_gazebo.py dep_4_limpio.yaml --cierres dep_4_limpio_cierres.png \
    --nombre dep_4 --salida gz --altura 1.0 --modo-cierres cristal
```
- Si queda alguna fuga, añade un segmento `[x0, y0, x1, y1]` (en píxeles, como los muestra GIMP) a `cortes_dep_4.yaml`.
- `--modo-cierres invisible` deja los cierres solo como colisión: el robot choca con ellos pero el gpu_lidar no los ve, como pasa con el cristal real.
