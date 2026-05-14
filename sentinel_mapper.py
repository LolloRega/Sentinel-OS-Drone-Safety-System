import numpy as np
import open3d as o3d
import copy
import time

def draw_registration_result(source, target, transformation):
    """
    Visualizza l'allineamento. 
    Source (La Scansione Attuale) sarà ROSSA.
    Target (La Mappa Precedente) sarà BLU.
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    
    source_temp.paint_uniform_color([1.0, 0.0, 0.0]) # Rosso
    target_temp.paint_uniform_color([0.0, 0.0, 1.0]) # Blu
    
    # Applichiamo la matrice spaziale calcolata per sovrapporli
    source_temp.transform(transformation)
    
    o3d.visualization.draw_geometries([source_temp, target_temp], 
                                      window_name="Sentinel SLAM - Allineamento ICP")

def create_collapsed_room():
    """Genera una finta stanza (un angolo tra due muri)"""
    wall1 = o3d.geometry.TriangleMesh.create_box(width=5.0, height=3.0, depth=0.2)
    wall2 = o3d.geometry.TriangleMesh.create_box(width=0.2, height=3.0, depth=5.0)
    room = wall1 + wall2
    pcd = room.sample_points_uniformly(number_of_points=8000)
    return pcd

if __name__ == "__main__":
    print("=== SENTINEL OS: MODULO SLAM & ODOMETRIA ===")
    
    # 1. Creiamo la Mappa Globale (Ciò che il drone ha visto al secondo precedente)
    target_pcd = create_collapsed_room()
    
    # 2. Simoliamo il Movimento Segreto del drone (L'Odometria Reale che dobbiamo scoprire)
    # Facciamo muovere il drone di: +1.2m avanti (X), +0.5m lato (Y), e lo facciamo ruotare di 10 gradi
    source_pcd = copy.deepcopy(target_pcd)
    
    true_movement = np.eye(4)
    true_movement[0, 3] = 1.2  # Spostamento X
    true_movement[1, 3] = 0.5  # Spostamento Y
    theta = np.radians(10)     # Rotazione (Imbardata)
    true_movement[0, 0] = np.cos(theta); true_movement[0, 1] = -np.sin(theta)
    true_movement[1, 0] = np.sin(theta); true_movement[1, 1] = np.cos(theta)
    
    # Applichiamo il movimento alla nuova scansione (si sposterà fisicamente)
    source_pcd.transform(true_movement)
    
    # --- LA SFIDA PER IL COMPUTER ---
    print("\n[SISTEMA] Avvio algoritmo ICP (Point-to-Point)...")
    start_time = time.time()
    
    # Diamo al sistema una tolleranza massima di ricerca di 2 metri
    threshold = 2.0 
    
    # Il computer parte dal presupposto di non essersi mosso (Matrice Identità)
    initial_guess = np.eye(4) 
    
    # ESECUZIONE DELLA MATEMATICA
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, initial_guess,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )
    
    calc_time = time.time() - start_time
    
    # --- RISULTATI DELL'ODOMETRIA ---
    print(f"\n[ICP] Ottimizzazione convergente in {calc_time:.4f} secondi")
    print(f"[ICP] Fitness (Punti allineati): {reg_p2p.fitness * 100:.2f}%")
    
    # Estraiamo l'odometria calcolata dall'algoritmo (Matrice Inversa dello spostamento)
    odometry = np.linalg.inv(reg_p2p.transformation)
    
    print("\n=== ODOMETRIA ESTRATTA (Dove si trova il drone) ===")
    print(f" -> Spostamento Calcolato Asse X: {odometry[0, 3]:.3f} metri")
    print(f" -> Spostamento Calcolato Asse Y: {odometry[1, 3]:.3f} metri")
    print("===================================================\n")
    
    print("[INFO] Apertura del render 3D.")
    print("Vedrai le due nuvole di punti fonderci perfettamente in un'unica mappa viola scuro.")
    
    draw_registration_result(source_pcd, target_pcd, reg_p2p.transformation)