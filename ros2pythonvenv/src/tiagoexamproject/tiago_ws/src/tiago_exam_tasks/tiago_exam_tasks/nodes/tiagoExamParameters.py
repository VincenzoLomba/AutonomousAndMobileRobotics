
# This file contains some parameters that are gonna be used in developing the Project.
# BE AWARE: these parameters exist only to make development easier!

gazeboWorldName = "group7"

fsmTimerPeriodParameterName = "fsm_timer_period"
fsmTimerPeriodParameterDefaultValue = 0.5
mapSavePathParameterName = "map_path"
mapSavePathParameterDefaultValue = "/home/vincenzo/Documenti/AMR/ros2pythonvenv/src/tiagoexamproject/map" # "/tmp/tiagomap/"
mapSaveNameParameterName = "map_name"
mapSaveNameParameterDefaultValue = "map"

nav2SaveMapServiceName = "/map_saver/save_map"
nav2MapTopic = "map"

tasksFSMFolderName = "tiago_exam_tasks"
task1FSMNodeName = "task1_fsm_node"
task2FSMNodeName = "task2_fsm_node"
task3FSMNodeName = "task3_fsm_node"
task1FSMlaunchDelay = 15.0
task2FSMlaunchDelay = 15.0
task3FSMlaunchDelay = 15.0

exploreLiteNodeName = "explore_node"
exploreLiteExploreResumeTopicLabel = "explore/resume"
exploreLiteStatusTopicLabel = "explore/status"

tiagoArmNodeName = "tiago_arm_node"
tiagoArmActionName = "tiago_arm_action"

HOME_JOINT_POSITIONS = [
    0.0,       # torso_lift_joint   0.0m
    2.478368,  # arm_1_joint        142°
    -1.343904, # arm_2_joint        -77°
    0.506145,  # arm_3_joint        29°
    1.972222,  # arm_4_joint        113°
    -1.745329, # arm_5_joint        -100°
    1.396263,  # arm_6_joint        80°
    0.226893,  # arm_7_joint        13°
]