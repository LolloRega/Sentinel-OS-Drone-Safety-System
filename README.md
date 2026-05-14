# Sentinel OS: Drone Search & Rescue Safety System

**Sentinel OS** è un sistema operativo modulare progettato per droni da soccorso operanti in ambienti indoor critici (edifici crollati, tunnel) dove il segnale GPS è assente. Il progetto implementa il concetto di **Shared Autonomy**, interponendo uno strato di intelligenza artificiale geometrica tra i comandi del pilota e i motori.

## 🚀 Moduli Principali

*   **E-Shield (Emergency Shield):** Sistema di *Collision Avoidance* attivo. Utilizza strutture dati **KD-Tree** per processare nuvole di punti LiDAR a 100Hz, implementando una frenata proporzionale asintotica che impedisce l'impatto fisico indipendentemente dall'input dell'utente.
*   **SLAM Mapper:** Modulo di localizzazione e mappatura simultanea. Utilizza l'algoritmo **ICP (Iterative Closest Point)** per calcolare l'odometria tramite matrici di rototraslazione 4x4, permettendo al drone di stimare la propria posizione con precisione millimetrica senza GPS.
*   **Target Vision:** Pipeline di riconoscimento vittime. Sfrutta il clustering **DBSCAN** per isolare oggetti tra le macerie e applica filtri biometrici dimensionali per identificare sagome umane svenute.

## 🛠️ Tech Stack
* **Language:** Python 3.x
* **3D Engine:** Open3D (Point Cloud Processing)
* **Math & Physics:** NumPy, SciPy (Linear Algebra & KD-Trees)

## 📈 Engineering Impact
Questo progetto integra concetti avanzati di **Meccanica Applicata** (cinematica dei corpi rigidi) e **Sistemi Integrati di Produzione**, simulando un ambiente di controllo real-time per la gestione di emergenze.
