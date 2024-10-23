^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot4_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2024-10-23)
------------------
* Fix the OakD launch so camera topics are properly namespaced when using a non-empty robot namespace
* Contributors: Chris Iverach-Brereton

2.0.0 (2024-09-25)
------------------
* Enable power-saver by default
* Add a delay to the oakd startup to work around a bug where the camera doesn't reliably publish data after restarting the systemd job
* Fix the resolution for the OakD config files
* Enable respawn for the republisher node
* Clean up config file formatting
* Remove unused variables
* Remove unused imports
* Disable XML linter; it's timing out and isn't needed for this package
* Update the reamappings for the stamped/unstamped topics
* Always use the Create3 republisher node
* Remove deprecations in robot.launch.py
* Enable TwistStamped publications for the teleop node
* Add joy_device launch argument in case js0 isn't the correct joystick
* Contributors: Chris Iverach-Brereton

1.0.3 (2024-07-02)
------------------
* Launch the create3 republisher only for discovery server
* Contributors: Hilary Luo

1.0.2 (2023-11-08)
------------------
* Merge pull request <https://github.com/turtlebot/turtlebot4_robot/issues/24>
  Fix: Oak-D parameters to use custom config file and Fixed launch file error
* Contributors: Harish Kumar Balaji, Roni Kreinin

1.0.1 (2023-03-22)
------------------

1.0.0 (2023-02-21)
------------------
* Updates for new DepthAI node.
* Updated turtlebot4_node config
* Namespacing
* Flake8 fixes
* Linter fixes
* Enable auto_standby for RPLiDAR.
* Enable or disable diagnostics based on TURTLEBOT4_DIAGNOSTICS environment variable.
* Launch joy_teleop_launch.py in Turtlebot4 Lite.
* Contributors: Joey Yang, Roni Kreinin

0.1.3 (2022-09-15)
------------------
* Added RPLIDAR Motor function to config
* Contributors: Roni Kreinin

0.1.2 (2022-05-30)
------------------
* Removed cyclonedds uri from launch files
* Contributors: Roni Kreinin

0.1.1 (2022-05-24)
------------------

0.1.0 (2022-05-24)
------------------
* First Galactic release
* Contributors: Roni Kreinin
