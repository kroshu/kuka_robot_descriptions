### Before submitting this PR into master please make sure:
If you added a new robot model:
- [ ] you extended the table of verified data in `README.md` with the new model
- [ ] you extended the CMakeLists.txt of the appropriate moveit configuration package with the new model
- [ ] you added a `test_<robot_model>.launch.py` and after launching the robot was visible in `rviz`
- [ ] you added a `<robot_model>_joint_limits.yaml` file in the `config` directory (to provide moveit support)

If you modified an already existing robot model:
- [ ] you checked and optionally updated the table of verified data in `README.md` with the changes
- [ ] you have run the `test_<robot_model>.launch.py` and the robot was visible in `rviz`

### Short description of the change
