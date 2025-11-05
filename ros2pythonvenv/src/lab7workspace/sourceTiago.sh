
#!/bin/bash
# ====================================================
# Script to build and prepare the workspace for Tiago
# Usage: source nomefile.sh
# ====================================================

# Salva la directory corrente
CURRENT_DIR=$(pwd)

echo "Entering ros2workspace..."
cd ros2workspace/ || { echo "Error: folder ros2workspace not found."; return 1; }

echo "Executing colcon build in ros2workspace..."
colcon build || { echo "❌ Errore durante il build di ros2workspace."; return 1; }

echo "📦 Sorgo install/setup.bash..."
source install/setup.bash

# Torna alla directory iniziale
cd "$CURRENT_DIR" || return 1

echo "👉 Entrando in tiagoworkspace..."
cd tiagoworkspace/ || { echo "❌ Errore: cartella tiagoworkspace non trovata."; return 1; }

echo "🔧 Eseguo colcon build in tiagoworkspace..."
colcon build || { echo "❌ Errore durante il build di tiagoworkspace."; return 1; }

echo "📦 Sorgo install/setup.bash..."
source install/setup.bash

# Torna alla directory iniziale
cd "$CURRENT_DIR" || return 1

echo ""
echo "✅ Build completata per entrambi i workspace!"
echo ""
echo "💡 Comandi utili da eseguire ora:"
echo "-----------------------------------"
echo "ros2 launch tiago_gazebo tiago_gazebo.launch.py"
echo "ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller"
echo "-----------------------------------"

