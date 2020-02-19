- Allow using both TransformStamped and PoseStamped for pose messages.
- Make the `T_BC` supereight parameter actually do something. Maybe update
  supereight as well to unify `init_pose` and `T_BC`.
- Re-enable map visualization. This will require changes to supereight to be
  efficient, but a slow version that visualizes the whole map each time should
  be feasible without changing supereight.

