# Tiago Exam Project — MATERIALE DI PARTENZA FORNITO DAL PROFESSORE

## 📁 Struttura della cartella principale

La cartella `tiagoexamproject` contiene tre elementi fondamentali:

### 1. `ros2_ws`
Workspace ROS 2 base.

Scopo:
- Fornire l’ambiente ROS 2 generale.

Utilizzo:
1. Source ROS di sistema (es. tramite `sourceros.sh`)
2. Build:
   ```
   colcon build
   ```
3. Source:
   ```
   source install/setup.bash
   ```

---

### 2. `tiago_ws`
Workspace specifico per Tiago.

Scopo:
- Contiene tutti i package necessari per Tiago
- Contiene anche i package dell’esame e quelli che verranno sviluppati

Utilizzo:
1. Dopo aver sourceato ROS e `ros2_ws`
2. Build:
   ```
   colcon build
   ```
3. Source:
   ```
   source install/setup.bash
   ```

---

### 3. `Tiago Exam Project Guide.pdf`
Documento ufficiale dell’esame.

Contiene:
- Specifiche del progetto
- Task richiesti
- Modalità di esecuzione
- Linee guida architetturali

---

## ⚙️ Preparazione ambiente

Ordine corretto:

1. Source ROS di sistema
   ```
   source sourceros.sh
   ```

2. Source `ros2_ws`
3. Source `tiago_ws`

---

## 📦 Package forniti per l’esame

All’interno di:

```
tiago_ws/src/
```

sono presenti due package fondamentali:

- `tiago_exam_worlds`
- `tiago_exam`

---

## 🌍 Package: `tiago_exam_worlds`

Contiene il mondo Gazebo dell’esame.

### Launch file

#### `pal_gazebo.launch.py`
- Launcher Gazebo di base
- Avvia:
  - `gzserver`
  - `gzclient`
- Utilizza un mondo predefinito (non parametrico)

#### `pal_gazebo_exam.launch.py`
- Launcher Gazebo specifico per l’esame
- Parametrico su:
  - `world_name`
- Carica:
  - `<world_name>.world`
- Avvia:
  - `gzserver`
  - `gzclient`
- Imposta i path dei modelli dell’esame

👉 Questo è il launcher corretto da usare per il progetto d’esame

---

## 🤖 Package: `tiago_exam`

Contiene il launcher principale dell’ambiente d’esame.

### Launch file

#### `tiago_spawn.launch.py`
- Si occupa esclusivamente dello spawn del robot
- Usa:
  - `gazebo_ros/spawn_entity.py`
- Inserisce Tiago in Gazebo
- Non gestisce altro

---

#### `tiago_exam.launch.py`
Launcher completo dell’ambiente d’esame.

Responsabilità:

1. Avvio Gazebo
   - Include:
     - `pal_gazebo_exam.launch.py`
   - Carica il mondo corretto

2. Spawn robot
   - Include:
     - `tiago_spawn.launch.py`

3. Bringup robot
   - Include:
     - `tiago_bringup.launch.py`
   - Attiva:
     - robot_description
     - TF
     - stato robot
     - componenti ROS necessari

4. MoveIt (opzionale, default attivo)
   - Include:
     - `move_group.launch.py`
   - Default:
     - `moveit:=true`

5. Posizionamento iniziale braccio
   - Lancia:
     - script `tuck_arm.py`
   - Posiziona il braccio nella configurazione "home"

---

## 🦾 Script: `tuck_arm.py`

Percorso:
```
tiago_exam/scripts/tuck_arm.py
```

Funzione:
- Usa `play_motion2`
- Invia un’azione per eseguire la motion `"home"`
- Porta il braccio in posizione iniziale
- Verifica disponibilità del sistema prima di eseguire
- Ritenta fino a 5 volte

---

## 🧠 Note architetturali

- `tiago_exam_worlds` → definisce il mondo
- `tiago_exam` → orchestra l’intero ambiente
- I launcher dell’esame:
  - NON vanno ignorati
  - costituiscono la base del progetto

---

## 🚀 Direzione per Task 1

Il launcher del Task 1:

- includerà:
  - `tiago_exam.launch.py`
- aggiungerà:
  - Nav2 + SLAM
  - explore_lite
  - FSM

Questo approccio replica la struttura già usata per TurtleBot, adattata a Tiago.
