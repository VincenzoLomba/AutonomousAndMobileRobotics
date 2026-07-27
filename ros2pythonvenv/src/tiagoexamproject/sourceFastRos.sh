
########################################################################################
##                                                                                    ##
##                              FASTROS LIBRARY COMMANDS                              ##
##                                                                                    ##
########################################################################################
#
# Important: if needed, recall to source this file automatically in the "~/.bashrc" file!
# Usually, a variable such as FASTROS_SOURCE="/../sourceFastRos.sh" is properly defined and then used to source this file (named indeed "sourceFastRos.sh")!
# IMPORTANT NOTE: this version of the FastRos Library is customized to be in particular used within the project folder "tiagoexamproject"
# Code and have fun! Dear greetings from Vins Lombardi

# fastrosEcho:
# A simple standardized colored message printer
# Supported FLAGS: INFO / WARN / ERROR / LINK
#  Example usage:
#    fastrosEcho INFO  "Starting build process..."
#    fastrosEcho WARN  "Missing dependency detected!"
#    fastrosEcho ERROR "Build failed!"
#    fastrosEcho LINK  "Documentation:" "https://example.com/docs"
#  Output colors (one for each FLAG):
#    INFO  → Green
#    WARN  → Yellow
#    ERROR → Red
#    LINK  → Cyan

fastrosEcho(){

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

# recode:
# Moves into the "~/Documenti/AMR/ros2pythonvenv/src/tiagoexamproject" directory.
# If successful, runs rossource, sources all what is related to Tiago usage, then launches VS Code (only if it's not already running).
# Uses fastrosEcho for standardized log output.

recode() {
    local target_dir="$HOME/Documenti/AMR/ros2pythonvenv/src/tiagoexamproject"

    if [ -d "$target_dir" ]; then
        cd "$target_dir" || { fastrosEcho ERROR "Failed to move to the directory '$target_dir'."; return 1; }
        rossource || return 1

        # A very simple couple of lines to also source tiago-related stuff!
        if [[ -f "sourceTiago.sh" ]]; then
            fastrosEcho INFO "Sourcing Tiago environment (sourceTiago.sh)..."
            source sourceTiago.sh || return 1
        else
            fastrosEcho WARN "sourceTiago.sh not found in the 'tiagoexamproject' folder! Tiago will NOT be sourced."
        fi

        # A very simple couple of lines to also launch VSCode!
        cd ..
        if pgrep -x "code" >/dev/null; then
            fastrosEcho INFO "VSCode is already running, you're now ready to work!"
        else
            fastrosEcho INFO "Now launching VSCode in the workspace src folder..."
            code .
        fi
        wmctrl -r :ACTIVE: -b add,maximized_vert,maximized_horz # Maximizing VSCode
    else
        fastrosEcho ERROR "Unable to execute: this method is intended to navigate to the '$target_dir' folder, which appears not to exist!"
    fi
}

# rossource:
# If invoked from a path containing "tiagoexamproject", automatically locates the project root, sources its sourceEnv.sh file, and then restores the starting directory

rossource(){
  

  # Save the current directory
  CURRENT_DIR=$(pwd)

  # Check if the current path contains "tiagoexamproject"
  if [[ "$CURRENT_DIR" == *"tiagoexamproject"* ]]; then

      # Try to detect tiagoexamproject root directory dynamically
      PROJECT_ROOT=$(echo "$CURRENT_DIR" | grep -o ".*/tiagoexamproject")

      # Verify project root directory was found
      if [ -z "$PROJECT_ROOT" ]; then
          fastrosEcho ERROR "Unable to detect 'tiagoexamproject' root directory! Make sure you are inside a folder which is a subfolder of an 'tiagoexamproject' one (or which is the folder itself)!"
          return 1
      fi

      # Move to the detected project directory
      cd "$PROJECT_ROOT" || {
          fastrosEcho ERROR "Detected a project directory which has resulted as not accessible: $PROJECT_ROOT"
          return 1
      }

      # Source the ROS2 Humble with the proper script
      if [ -f "sourceEnv.sh" ]; then
          fastrosEcho INFO "Sourcing ROS2 Humble from script $PROJECT_ROOT/sourceEnv.sh..."
          source sourceEnv.sh
      else
          fastrosEcho ERROR "Unexpected problem: sourceEnv.sh file not found in the tiagoexamproject folder!"
          return 1
      fi

      # Return to the original directory
      cd "$CURRENT_DIR" || return 1

      fastrosEcho INFO "ROS2 Humble environment successfully sourced!"

  else
      fastrosEcho ERROR "Unable to execute: the current path must contain a 'tiagoexamproject' folder! Make sure you are inside a folder which is a subfolder of an 'tiagoexamproject' one (or which is the folder itself)!"
      return 1
  fi
}

# resource
# If invocked within a proper folder (a path containing "tiagoexamproject"), automatically configure the ROS2 Humble environment (issuing rossource) AND also sourcing the local build (IF an install folder is present in the current path)

resource(){

  rossource

  # Source the generated setup file (if present)
  if [ -f "install/setup.bash" ]; then
      fastrosEcho INFO "Now sourcing install/setup.bash..."
      source install/setup.bash
  else
      fastrosEcho WARN "install/setup.bash not found, so no local source is gonna be performed!"
  fi
}

# recolcon
# If invoked from a path containing "tiagoexamproject" and ALSO "src" as an IMMEDIATE subfolder, automatically configure the ROS2 Humble environment (issuing rossource), then building with colcon and sourcing what obtained

recolcon(){

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

