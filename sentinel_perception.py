import numpy as np
import open3d as o3d
import time

# Importiamo il cervello che hai scritto prima!
from e_shield import SentinelFlightController

def generate_synthetic_environment():
    """Genera una nuvola di punti simulando un pilastro di cemento a 3.5 metri dal drone"""
    print("[SENSOR] Generazione ambiente 3D in corso...")
    
    # Creiamo un cilindro (raggio 0.5m, alto 4m)
    pillar = o3d.geometry.TriangleMesh.create_cylinder(radius=0.5, height=4.0)
    
    # Lo spostiamo a 3.5 metri davanti al drone (sull'asse X)
    pillar.translate([3.5, 0.0, 0.0])
    
    # Lo trasformiamo in una nuvola di punti (come farebbe un LiDAR reale)
    pcd = pillar.sample_points_uniformly(number_of_points=10000)
    
    # Diamo un colore al pilastro (es. grigio cemento)
    pcd.paint_uniform_color([0.6, 0.6, 0.6])
    return pcd

if __name__ == "__main__":
    print("=== SENTINEL OS: PERCEPTION & SHIELD LINK ===")
    
    # 1. Avviamo il nostro Controllore di Volo
    fc = SentinelFlightController(critical_dist=1.5, braking_dist=4.0)
    
    # 2. Otteniamo la scansione 3D dell'ambiente
    environment_pcd = generate_synthetic_environment()
    
    # 3. IL MOTORE DI RICERCA SPAZIALE: Costruiamo il KD-Tree
    start_time = time.time()
    pcd_tree = o3d.geometry.KDTreeFlann(environment_pcd)
    
    # Il drone si trova sempre nell'origine del suo stesso sensore
    drone_position = np.array([0.0, 0.0, 0.0])
    
    # Chiediamo al KD-Tree: trovami il SINGOLO punto (1) più vicino al drone
    [k, idx, _] = pcd_tree.search_knn_vector_3d(drone_position, 1)
    
    # Estraiamo le coordinate di quel punto esatto
    closest_point = np.asarray(environment_pcd.points)[idx[0]]
    
    # Calcoliamo la distanza lineare reale
    lidar_distance = np.linalg.norm(closest_point - drone_position)
    search_time = time.time() - start_time
    
    print(f"\n[KD-TREE] Punto di pericolo rilevato a coordinate: X:{closest_point[0]:.2f}, Y:{closest_point[1]:.2f}, Z:{closest_point[2]:.2f}")
    print(f"[KD-TREE] Distanza radiale calcolata: {lidar_distance:.2f} metri")
    print(f"[KD-TREE] Tempo di calcolo su 10.000 punti: {search_time:.6f} secondi")
    
    print("\n--- ELABORAZIONE FLIGHT CONTROLLER ---")
    pilot_velocity = 5.0 # Il pilota sta chiedendo di avanzare a 5 m/s
    safe_v = fc.compute_safe_velocity(pilot_velocity, lidar_distance)
    
    # --- VISUALIZZAZIONE DEL SISTEMA ---
    # Creiamo una sfera rossa gigante per marcare il punto di impatto calcolato dal KD-Tree
    impact_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
    impact_marker.translate(closest_point)
    impact_marker.paint_uniform_color([1.0, 0.0, 0.0])
    
    # Creiamo una sfera verde per mostrare dove si trova il drone
    drone_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
    drone_marker.translate(drone_position)
    drone_marker.paint_uniform_color([0.0, 1.0, 0.0])
    
    print("\n[INFO] Apertura interfaccia visiva. Verde = Drone, Grigio = Muro, Rosso = Punto di Impatto.")
    o3d.visualization.draw_geometries([environment_pcd, impact_marker, drone_marker], window_name="Sentinel 3D Perception")