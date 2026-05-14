import numpy as np
import open3d as o3d
import time
import copy

# Importiamo il modulo di sicurezza che hai scritto in precedenza!
from e_shield import SentinelFlightController

def get_lidar_scan(wall_distance):
    """Simula una scansione LiDAR 3D di un muro a una certa distanza"""
    wall = o3d.geometry.TriangleMesh.create_box(width=5.0, height=3.0, depth=0.2)
    wall.translate([wall_distance, 0.0, 0.0])
    return wall.sample_points_uniformly(number_of_points=1000)

def run_slam_icp(source_pcd, target_pcd):
    """Esegue l'algoritmo ICP per l'Odometria"""
    reg = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, max_correspondence_distance=1.0, init=np.eye(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
    )
    # Estraiamo lo spostamento sull'asse X
    odometry_x = np.linalg.inv(reg.transformation)[0, 3]
    return odometry_x

if __name__ == "__main__":
    print("=== AVVIO SENTINEL OS: INTEGRAZIONE SHIELD & SLAM ===")
    
    # Inizializziamo il Controllore di Volo (Arresto a 1.5m, Frenata da 4.0m)
    fc = SentinelFlightController(critical_dist=1.5, braking_dist=4.0)
    
    # Parametri di Missione
    pilot_velocity = 2.0  # Il pilota chiede 2 m/s costanti
    dt = 0.5              # Il nostro "Tick" di sistema (mezzo secondo)
    
    # La realtà fisica (nascosta al computer)
    true_distance_to_wall = 8.0 
    
    # Memoria del Mapper
    previous_scan = get_lidar_scan(true_distance_to_wall)
    total_odometry = 0.0
    
    print("[MISSIONE] Decollo completato. Pilota richiede avanzamento veloce verso l'ignoto.\n")
    
    for tick in range(1, 15):
        print(f"--- [TICK {tick:02d} | Tempo: {tick*dt:.1f}s] ---")
        
        # 1. ACQUISIZIONE SENSORI
        current_scan = get_lidar_scan(true_distance_to_wall)
        
        # 2. ELABORAZIONE E-SHIELD (Uso del KD-Tree per trovare il punto più vicino)
        pcd_tree = o3d.geometry.KDTreeFlann(current_scan)
        [k, idx, _] = pcd_tree.search_knn_vector_3d(np.array([0.0, 0.0, 0.0]), 1)
        lidar_distance = np.linalg.norm(np.asarray(current_scan.points)[idx[0]])
        
        # Il Flight Controller decide la velocità sicura
        safe_velocity = fc.compute_safe_velocity(pilot_velocity, lidar_distance)
        
        # 3. ESECUZIONE MOVIMENTO FISICO
        if safe_velocity > 0:
            step_distance = safe_velocity * dt
            true_distance_to_wall -= step_distance
            print(f"[MOTORI] Eseguito avanzamento fisico di {step_distance:.2f}m")
        else:
            print("[MOTORI] OVERRIDE ATTIVO. Motori bloccati. Il drone è in Hovering sicuro.")
            
        # 4. ELABORAZIONE SLAM / ODOMETRIA (Calcoliamo di quanto ci siamo mossi)
        # Rifacciamo la scansione DOPO esserci mossi
        new_scan = get_lidar_scan(true_distance_to_wall)
        step_odometry = run_slam_icp(new_scan, previous_scan)
        
        total_odometry += step_odometry
        previous_scan = copy.deepcopy(new_scan)
        
        print(f"[MAPPER] Spostamento rilevato: {step_odometry:.2f}m | Odometria Totale: {total_odometry:.2f}m")
        print("-" * 50)
        
        time.sleep(0.5) # Pausa per leggere il terminale
        
    print("\n=== FINE MISSIONE ===")