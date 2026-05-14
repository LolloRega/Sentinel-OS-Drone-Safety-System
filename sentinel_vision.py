import numpy as np
import open3d as o3d
import random

def generate_disaster_scene():
    """Genera macerie casuali e una 'vittima' nascosta per il test di visione"""
    scene = o3d.geometry.PointCloud()
    
    # 1. Generiamo blocchi di macerie (mura, travi, detriti)
    for _ in range(6):
        w, h, d = random.uniform(0.4, 2.2), random.uniform(0.4, 1.8), random.uniform(0.4, 2.2)
        rubble = o3d.geometry.TriangleMesh.create_box(width=w, height=h, depth=d)
        # Distribuzione casuale in un'area di 8x8 metri
        rubble.translate([random.uniform(-4, 4), random.uniform(-4, 4), 0])
        scene += rubble.sample_points_uniformly(2500)
        
    # 2. Generiamo la "Vittima" (Geometria con proporzioni umane)
    # Cilindro steso: raggio 0.25m (spessore corpo), altezza 1.75m (statura)
    victim = o3d.geometry.TriangleMesh.create_cylinder(radius=0.22, height=1.70)
    
    # Rotazione per simulare la posizione supina (stesa a terra)
    R = victim.get_rotation_matrix_from_xyz((0, np.pi/2, 0))
    victim.rotate(R, center=(0,0,0))
    
    # Posizionamento della vittima (leggermente rialzata da terra per il volume)
    victim.translate([random.uniform(-2, 2), random.uniform(-2, 2), 0.15])
    
    scene += victim.sample_points_uniformly(4000)
    
    # Colore base grigio (macerie/polvere)
    scene.paint_uniform_color([0.4, 0.4, 0.4])
    return scene

if __name__ == "__main__":
    print("=== SENTINEL OS: MODULO VISIONE INTELLIGENTE ===")
    
    # Acquisizione dati ambiente
    pcd = generate_disaster_scene()
    print(f"[SENSOR] Scansione completata: {len(pcd.points)} punti analizzati.")
    
    # --- STEP 1: CLUSTERING (PARAMETRI CALIBRATI) ---
    print("[IA] Analisi spaziale DBSCAN in corso...")
    # eps=0.25 (25cm) permette di unire i punti del corpo senza fondersi con detriti lontani
    labels = np.array(pcd.cluster_dbscan(eps=0.25, min_points=40, print_progress=False))
    max_label = labels.max()
    print(f"[IA] Identificati {max_label + 1} cluster indipendenti.")
    
    victims_found = 0
    geometries_to_draw = [pcd]
    
    # --- STEP 2: ANALISI BIOMETRICA ---
    for i in range(max_label + 1):
        indices = np.where(labels == i)[0]
        cluster_cloud = pcd.select_by_index(indices)
        
        # Calcolo della Bounding Box (scatola di ingombro)
        bbox = cluster_cloud.get_axis_aligned_bounding_box()
        bbox.color = (0.5, 0.5, 0.5) # Colore neutro per oggetti scartati
        
        extent = bbox.get_extent()
        length = max(extent)            # Dimensione maggiore
        height = extent[2]              # Altezza da terra (Z)
        width = sorted(extent)[1]       # Dimensione trasversale
        
        # Filtri Biometrici (Standard per un essere umano steso)
        # Lunghezza: 1.5m - 2.0m | Larghezza: 0.3m - 0.9m | Altezza: < 0.5m
        match_len = 1.45 <= length <= 1.95
        match_wid = 0.35 <= width <= 0.85
        match_hei = 0.10 <= height <= 0.55
        
        if match_len and match_wid and match_hei:
            print(f"\n[!!!] TARGET ACQUISITO: Oggetto {i}")
            print(f"    > Geometria compatibile con sagoma umana.")
            print(f"    > Dimensioni: {length:.2f}m x {width:.2f}m (H: {height:.2f}m)")
            
            # Evidenziazione nel visore
            np.asarray(pcd.colors)[indices] = [0.0, 1.0, 0.0] # Verde brillante per la vittima
            bbox.color = (0, 1, 0) # Box verde
            victims_found += 1
        
        geometries_to_draw.append(bbox)
        
    print(f"\n=== REPORT FINALE: {victims_found} vittima/e localizzata/e. ===")
    
    # Visualizzazione
    print("[INFO] Apertura interfaccia 3D Sentinel...")
    o3d.visualization.draw_geometries(geometries_to_draw, 
                                      window_name="Sentinel Vision - Search & Rescue",
                                      width=1280, height=720)