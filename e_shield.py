import time

class SentinelFlightController:
    def __init__(self, critical_dist=1.5, braking_dist=4.0):
        """
        Inizializza il Flight Controller con i parametri di sicurezza spaziale.
        critical_dist: Distanza minima assoluta (in metri). Il drone si ferma.
        braking_dist: Distanza a cui inizia l'intervento del computer (in metri).
        """
        self.critical_dist = critical_dist
        self.braking_dist = braking_dist

    def compute_safe_velocity(self, pilot_v, lidar_dist):
        """
        Calcola la velocità reale da inviare ai motori basata sull'input del pilota 
        e sui dati spaziali del LiDAR.
        """
        # CASO 1: Collisione imminente (Zona Rossa)
        if lidar_dist <= self.critical_dist:
            print(f"[! ALLARME !] Ostacolo a {lidar_dist:.2f}m. OVERRIDE TOTALE: Frenata d'emergenza.")
            return 0.0
            
        # CASO 2: Ostacolo in avvicinamento (Zona Gialla)
        elif lidar_dist <= self.braking_dist:
            # Calcoliamo quanta percentuale di frenata applicare.
            # È un'interpolazione lineare tra la distanza critica e la distanza di frenata.
            available_space = lidar_dist - self.critical_dist
            braking_zone = self.braking_dist - self.critical_dist
            
            # Fattore da 0.0 (fermo) a 1.0 (massima velocità)
            allowed_ratio = available_space / braking_zone 
            
            # La velocità sicura è una frazione di quella richiesta dal pilota
            max_safe_v = pilot_v * allowed_ratio
            
            # Applichiamo il limite
            actual_v = min(pilot_v, max_safe_v)
            print(f"[SHIELD] Ostacolo a {lidar_dist:.2f}m. Intervento attivo: Taglio potenza al {allowed_ratio*100:.0f}%. V_Out: {actual_v:.2f} m/s")
            return actual_v
            
        # CASO 3: Spazio libero (Zona Verde)
        else:
            print(f"[OK] Spazio libero ({lidar_dist:.2f}m). Ubbidienza totale al pilota. V_Out: {pilot_v:.2f} m/s")
            return pilot_v

# ==========================================
# SIMULAZIONE DEL TEST DI VOLO IN FABBRICA
# ==========================================
if __name__ == "__main__":
    print("=== AVVIO SENTINEL OS: TEST COLLISION AVOIDANCE ===")
    fc = SentinelFlightController(critical_dist=1.5, braking_dist=5.0)
    
    # Immaginiamo che il pilota, in preda al panico o non vedendo nulla per il fumo,
    # spinga la leva in avanti al massimo, richiedendo 6.0 m/s costanti verso un muro.
    pilot_requested_velocity = 6.0 
    
    # Il drone si trova a 10 metri dal muro
    current_distance_to_wall = 10.0 
    
    dt = 0.5 # Intervallo di simulazione (mezzo secondo)
    
    for _ in range(15):
        # 1. Il computer legge il comando del pilota e il dato del LiDAR
        # 2. Il computer calcola la velocità sicura
        actual_velocity = fc.compute_safe_velocity(pilot_requested_velocity, current_distance_to_wall)
        
        # 3. Aggiorniamo la posizione fisica del drone nel mondo
        # Spazio = Velocità * Tempo
        current_distance_to_wall -= (actual_velocity * dt)
        
        time.sleep(0.3) # Rallentiamo l'output per renderlo leggibile
        
    print("=== SIMULAZIONE TERMINATA ===")