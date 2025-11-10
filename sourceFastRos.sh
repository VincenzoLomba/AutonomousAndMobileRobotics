
########################################################################################
##                                                                                    ##
##                              FASTROS LIBRARY COMMANDS                              ##
##                                                                                    ##
########################################################################################
#
# Important: if needed, recall to source this file automatically in the "~/.bashrc" file!
# Usually, a variable such as FASTROS_SOURCE="/../sourceFastRos.sh" is properly defined and then used to source this file! (named indeed "sourceFastRos.sh")

fastrosEcho(){
  # ====================================================
  #  fastrosEcho - Standardized colored message printer
  #  Supports: INFO / WARN / ERROR / LINK
  # ====================================================
  #
  #  Example usage:
  #    fastrosEcho INFO  "Starting build process..."
  #    fastrosEcho WARN  "Missing dependency detected!"
  #    fastrosEcho ERROR "Build failed!"
  #    fastrosEcho LINK  "Documentation:" "https://example.com/docs"
  #
  #  Output colors:
  #    INFO  → Green
  #    WARN  → Yellow
  #    ERROR → Red
  #    LINK  → Cyan

  PREFIX="[fastros]"
  TIMESTAMP="[$(date '+%H:%M:%S')]"

  # Normalize the first argument
  LEVEL_TYPE=$(echo "$1" | tr '[:lower:]' '[:upper:]')

  # Define styles
  PREFIX_STYLE="\033[1;37m"  # Bold white for prefix
  NC="\033[0m"               # Reset color

  case "$LEVEL_TYPE" in
    INFO)
      LEVEL="[INFO]"
      MSG_COLOR="\033[0;32m"   # Green
      MESSAGE="$2"
      ;;
    WARN)
      LEVEL="[WARN]"
      MSG_COLOR="\033[0;33m"   # Yellow
      MESSAGE="$2"
      ;;
    ERROR|ERR)
      LEVEL="[ERR!]"
      MSG_COLOR="\033[0;31m"   # Red
      MESSAGE="$2"
      ;;
    LINK)
      LEVEL="[LINK]"
      MSG_COLOR="\033[0;36m"   # Cyan
      MESSAGE="$2"
      LINK_TEXT="$3"
      echo -e "${PREFIX_STYLE}${PREFIX}${TIMESTAMP}${LEVEL}${NC} ${MSG_COLOR}${MESSAGE}${LINK_TEXT}${NC}"
      return
      ;;
    *)
      LEVEL="[INFO]"
      MSG_COLOR="\033[0;32m"
      MESSAGE="$1"
      ;;
  esac

  echo -e "${PREFIX_STYLE}${PREFIX}${TIMESTAMP}${LEVEL}${NC} ${MSG_COLOR}${MESSAGE}${NC}"
}

# moveamr:
# Searches recursively for a directory named "AMR" inside the user's Documents folder.
# If found, changes the current working directory to the first match found (shortest path).
# If not found or the directory change fails, prints an error message using fastrosEcho.

moveamr() {
    local amr_path="$HOME/Documenti/AMR"

    if [ -d "$amr_path" ]; then
        cd "$amr_path" || fastrosEcho "Error: failed to change directory (even if found)!"
    else
        fastrosEcho "Error: directory '$amr_path' does not exist!"
    fi
}

# recode:
# Moves into the "~/Documenti/AMR/ros2pythonvenv/src" directory.
# If successful, runs rossource and then launches VS Code (only if it's not already running).
# Uses fastrosEcho for standardized log output.

recode() {
    local target_dir="$HOME/Documenti/AMR/ros2pythonvenv/src"

    if [ -d "$target_dir" ]; then
        cd "$target_dir" || { fastrosEcho ERROR "Failed to change directory."; return 1; }
        rossource
        if pgrep -x "code" >/dev/null; then
            fastrosEcho INFO "VSCode is already running, you're now ready to work!"
        else
            fastrosEcho INFO "Now launching VSCode in the workspace src folder..."
            code .
        fi
        wmctrl -r :ACTIVE: -b add,maximized_vert,maximized_horz
    else
        fastrosEcho ERROR "Directory '$target_dir' does not exist."
    fi
}

rossource(){
  # Creating custom command: rossource
  # Description: if invocked within a proper folder, automatically configure the ROS2 environment

  # Save the current directory
  CURRENT_DIR=$(pwd)

  # Check if the current path contains "AMR" and "src"
  if [[ "$CURRENT_DIR" == *"AMR"* && "$CURRENT_DIR" == *"src"* ]]; then

      # Try to detect AMR root directory dynamically
      AMR_ROOT=$(echo "$CURRENT_DIR" | grep -o ".*/AMR")

      # Verify AMR root directory was found
      if [ -z "$AMR_ROOT" ]; then
          fastrosEcho ERROR "Unable to detect AMR directory! Make sure you are inside a folder which is a subfolder of an AMR one."
          return 1
      fi

      fastrosEcho INFO "Requirements met, now setting up AMR environment..."

      # Move to the detected AMR directory
      cd "$AMR_ROOT" || { fastrosEcho ERROR "Detected AMR directory not accessible: $AMR_ROOT"; return 1; }

      # Source the environment setup script
      if [ -f "sourceEnv.sh" ]; then
          fastrosEcho INFO "Sourcing AMR environment from $AMR_ROOT/sourceEnv.sh..."
          source sourceEnv.sh
      else
          fastrosEcho ERROR "sourceEnv.sh file not found in the AMR folder ($AMR_ROOT)!"
          return 1
      fi

      # Return to the original directory
      cd "$CURRENT_DIR" || return 1

      fastrosEcho INFO "AMR environment successfully sourced."

  else
      fastrosEcho ERROR "Unable to execute: the current path must contain both 'AMR' and 'src'!"
      return 1
  fi
}


resource(){
  # Creating custom command: resource
  # Description: if invocked within a proper folder, automatically configure the ROS2 environment also sourcing the local build (if an install folder is present in the current path)

  fastrosEcho INFO "Setting up AMR environment using rossource..."
  rossource

  # Source the generated setup file (if present)
  if [ -f "install/setup.bash" ]; then
      fastrosEcho INFO "Now sourcing install/setup.bash..."
      source install/setup.bash
  else
      fastrosEcho WARN "install/setup.bash not found, so no local source is gonna be performed!"
  fi
}


recolcon(){
  # Creating custom command: recolcon
  # Description: if invocked within a proper folder, automatically configure the ROS2 environment, build with colcon and source what obtained

  # Save the current directory
  CURRENT_DIR=$(pwd)

  # Check if a subfolder named "src" exists in the current directory
  if [ -d "src" ]; then
      fastrosEcho INFO "Detected valid path for 'recolcon' to be called ($(pwd) contains a 'src' folder)..."
      rossource

      # Run colcon build
      fastrosEcho INFO "Running colcon build..."
      colcon build

      # Source the generated setup file
      if [ -f "install/setup.bash" ]; then
          fastrosEcho INFO "Now sourcing install/setup.bash..."
          source install/setup.bash
      else
          fastrosEcho WARN "install/setup.bash not found after build, so no local source is gonna be performed!"
      fi

  else
      fastrosEcho ERROR "Unable to execute: no 'src' subfolder found in the current path $(pwd)"
      fastrosEcho INFO "Make sure you are inside a folder meant to be built with 'colcon build'!"
      return 1
  fi
}

