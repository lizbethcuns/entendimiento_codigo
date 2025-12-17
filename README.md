#objetivo de la tarea
hay que generar 2 trayectorias cada una con 2 algoritmas diferentes en mi caso mi trayectoria principal es el LPA* y aparte udar en RRT para generar otros waitpoints y ver la diferencias entre el RRT CON EL lpa* Implementación de planificación con el algoritmo asignado por el docente con waypoints separados 0.5 metros y 1 metro.
# entendimiento_codigo´ESTE CODIGO SE ENCUENTRA EN Global_Planner/f1tenth/f1tenth_map.py
import os
import cv2
import csv
import yaml
import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils import Grid, SearchFactory
from pathlib import Path


def load_map(yaml_path, downsample_factor=1):
    yaml_path = Path(yaml_path)  # asegurar Path
    with yaml_path.open('r') as f:
        map_config = yaml.safe_load(f)


    img_path = Path(map_config['image'])
    if not img_path.is_absolute():
        img_path = (yaml_path.parent / img_path).resolve()
    map_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    resolution = map_config['resolution']
    origin = map_config['origin']

    # Binarizar: 1 = ocupado, 0 = libre
    map_bin = np.zeros_like(map_img, dtype=np.uint8)
    map_bin[map_img < int(0.45 * 255)] = 1

    # Engrosar obstáculos según el factor
    if downsample_factor > 12:
        map_bin = cv2.dilate(map_bin, np.ones((5, 5), np.uint8), iterations=2)
    elif downsample_factor >= 4:
        map_bin = cv2.dilate(map_bin, np.ones((3, 3), np.uint8), iterations=1)
    # para 1-3 no se dilata

    # Downsampling con interpolación adecuada
    map_bin = map_bin.astype(np.float32)
    h, w = map_bin.shape
    new_h, new_w = h // downsample_factor, w // downsample_factor
    map_bin = cv2.resize(map_bin, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # Re-binarizar según nivel
    if downsample_factor > 12:
        map_bin = (map_bin > 0.10).astype(np.uint8)
    elif downsample_factor >= 4:
        map_bin = (map_bin > 0.25).astype(np.uint8)
    else:
        map_bin = (map_bin >= 0.5).astype(np.uint8)

    # Ajustar resolución
    resolution *= downsample_factor

    return map_bin, resolution, origin


def grid_from_map(map_bin):
    h, w = map_bin.shape
    env = Grid(w, h)
    obstacles = {(x, h - 1 - y) for y in range(h) for x in range(w) if map_bin[y, x] == 1}
    env.update(obstacles)
    return env


def world_to_map(x_world, y_world, resolution, origin):
    x_map = int((x_world - origin[0]) / resolution)
    y_map = int((y_world - origin[1]) / resolution)
    return (x_map, y_map)


def map_to_world(x_map, y_map, resolution, origin, image_height):
    x_world = x_map * resolution + origin[0]
    y_world = y_map * resolution + origin[1]
    return (x_world, y_world)


def save_path_as_csv(path, filename, resolution, origin, image_height):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x_map, y_map in reversed(path):  # invertir para ir de start a goal
            x, y = map_to_world(x_map, y_map, resolution, origin, image_height)
            writer.writerow([x, y])


if __name__ == "__main__":
    HERE = Path(__file__).resolve().parent
    yaml_path = HERE.parent / "Mapas-F1Tenth" / "example_map.yaml"              #en la carpeta Mapas-F1Tenth estan todos los mapas,se cambia example_map.yaml por otro mapa que quiera
    downsample_factor = 8  # Ajusta este valor según lo que necesites           #se puede cambiar el numero para que sea mas rapido   

#-----------------------------linea en medio del mapa, mis puntos debe ser uno encima de la linea y uno debajo de la linea------------------------------------------------------------------

    x_start, y_start = 0.0, 1.0                                                    #posicion inicial del inicio 
    x_goal, y_goal = 0.0, -1.5
#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    map_bin, resolution, origin = load_map(yaml_path, downsample_factor)           #-------------------------ojo ----------------------
    env = grid_from_map(map_bin)

    start = world_to_map(x_start, y_start, resolution, origin)                     #-------------------------indices de una celda 
    goal = world_to_map(x_goal, y_goal, resolution, origin)

    print(f"Start (map): {start}, Goal (map): {goal}")                         
    planner = SearchFactory()("a_star", start=start, goal=goal, env=env)          #---------------se puede cambiar de planificador , en la carpeta Global_Planner/python_motion_planning/global_planner/graph_search(estan todos los algoritmos)
#---para saber como poner ese nombre de "a_star", start=start, goal=goal, env=env , en la carpeta   Global_Planner/global_examples.py
 # planner = search_factory("dijkstra", start=start, goal=goal, env=env)
    # planner = search_factory("gbfs", start=start, goal=goal, env=env)
    # planner = search_factory("theta_star", start=start, goal=goal, env=env)
    # planner = search_factory("lazy_theta_star", start=start, goal=goal, env=env)
    # planner = search_factory("s_theta_star", start=start, goal=goal, env=env)
    # planner = search_factory("jps", start=start, goal=goal, env=env)
    # planner = search_factory("d_star", start=start, goal=goal, env=env)
    # planner = search_factory("lpa_star", start=start, goal=goal, env=env)
    # planner = search_factory("d_star_lite", start=start, goal=goal, env=env)
  # -------------------- PARA LA TAREA --------------------------------
      '''
    sample search
    '''
    # # build environment
    # start = (18, 8)
    # goal = (37, 18)
    # env = Map(51, 31)
    # env = Map(51, 31)
    # obs_rect = [
    #     [14, 12, 8, 2],
    #     [18, 22, 8, 3],
    #     [26, 7, 2, 12],
    #     [32, 14, 10, 2]
    # ]
    # obs_circ = [
    #     [7, 12, 3],
    #     [46, 20, 2],
    #     [15, 5, 2],
    #     [37, 7, 3],
    #     [37, 23, 3]
    # ]
    # env.update(obs_rect=obs_rect, obs_circ=obs_circ)

    # # creat planner
    # planner = search_factory("rrt", start=start, goal=goal, env=env)
    # planner = search_factory("rrt_connect", start=start, goal=goal, env=env)
    # planner = search_factory("rrt_star", start=start, goal=goal, env=env)
    # planner = search_factory("informed_rrt", start=start, goal=goal, env=env)

    # # animation
    # planner.run()

    # ======================================================== ESTA PARTE NOSE SI TAMBIEN SEA DE PONER EN LA TAREA SI ES QUE SEA NECESARIO

    '''
    evolutionary search
    '''
    # planner = search_factory("aco", start=start, goal=goal, env=env)
    # planner = search_factory("pso", start=start, goal=goal, env=env)
    # planner.run()
  # -------------------- HASTA AQUI ES PARA LA TAREA --------------------------------

    planner.run()

    cost, path, _ = planner.plan()               # ---------- me lo devuelve ya echo 
    save_path_as_csv(path, "astar_path_real.csv", resolution, origin, map_bin.shape[0])             # ----------me crea el ccv con la rjta y se almacena en examples 
    print("Ruta guardada como astar_path_real.csv")
