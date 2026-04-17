# Exploration status constants
# EXPLORATION_STARTED: Published exactly once, on the very first resume() call ever received by the node
# EXPLORATION_IN_PROGRESS: Exploration resumed after pause
# EXPLORATION_PAUSED: Exploration manually stopped via /explore/resume topic, or node just started
# EXPLORATION_COMPLETE: No frontiers remaining, exploration finished
# RETURNING_TO_ORIGIN: Robot navigating back to initial pose (if return_to_init: true)
# RETURNED_TO_ORIGIN: Robot successfully reached initial pose
# RETURN_TO_ORIGIN_FAILED: Robot failed to return to initial pose (NavigateToPose aborted or canceled)

string EXPLORATION_STARTED=exploration_started
string EXPLORATION_IN_PROGRESS=exploration_in_progress
string EXPLORATION_PAUSED=exploration_paused
string EXPLORATION_COMPLETE=exploration_complete
string RETURNING_TO_ORIGIN=returning_to_origin
string RETURNED_TO_ORIGIN=returned_to_origin
string RETURN_TO_ORIGIN_FAILED=return_to_origin_failed

# Current exploration status
string status
