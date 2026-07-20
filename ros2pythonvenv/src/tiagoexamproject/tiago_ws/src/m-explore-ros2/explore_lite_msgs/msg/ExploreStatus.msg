# Exploration status constants — always kept 1:1 with the NodePhase internal enum in the "newer" version by Vins
#
# EXPLORATION_STARTED:              Published exactly once on the very first start of
#                                   exploration (via /explore/resume True or automatically
#                                   if start_exploration_immediately=true).
# EXPLORATION_IN_PROGRESS:          Exploration actively running (resumed after pause or
#                                   restarted after completion/return-to-init).
# EXPLORATION_PAUSED:               Exploration manually stopped via False on
#                                   /explore/resume, or node started with
#                                   start_exploration_immediately=false.
# EXPLORATION_PAUSED_DURING_RETURN: Return-to-init was interrupted by False on
#                                   /explore/resume. Publishing True resumes the return
#                                   from the current robot position.
# EXPLORATION_COMPLETE:             All frontiers exhausted AND (if return_to_init=true)
#                                   the robot has successfully returned to its initial pose.
# RETURNING_TO_ORIGIN:              Robot actively navigating back to initial pose
#                                   (only when return_to_init=true).
# RETURN_TO_ORIGIN_FAILED:          Return-to-init aborted or timed out. Publishing True
#                                   retries the return; False resets to EXPLORATION_PAUSED.

string EXPLORATION_STARTED=exploration_started
string EXPLORATION_IN_PROGRESS=exploration_in_progress
string EXPLORATION_PAUSED=exploration_paused
string EXPLORATION_PAUSED_DURING_RETURN=exploration_paused_during_return
string EXPLORATION_COMPLETE=exploration_complete
string RETURNING_TO_ORIGIN=returning_to_origin
string RETURN_TO_ORIGIN_FAILED=return_to_origin_failed

# Current exploration status
string status
