
# This file contains some parameters that are gonna be used in developing the nodes of the Project.
# BE AWARE: these parameters exist only to make development easier!

exploreLiteExploreResumeTopicLabel = "explore/resume"
exploreLiteStatusTopicLabel = "explore/status"
exploreLiteNodeName = "explore_node"

task1FSMNodeName = "task1_fsm_node"

nav2SaveMapServiceName = "/map_saver/save_map"
nav2MapTopic = "map"
mapSaveFullPathParameterName = "map_path_full"
mapSaveFullPathParameterDefaultValue = "" # "/tmp/map"

fsmTimerPeriodParameterName = "fsm_timer_period"
fsmTimerPeriodParameterDefaultValue = 0.5

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