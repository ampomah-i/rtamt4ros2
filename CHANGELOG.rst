^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtamt4ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2026-07-23)
------------------
* Pin a stable first-party flake8 configuration across ROS 2 distributions.
* Contributors: Immanuel Ampomah Mensah

0.2.0 (2026-07-23)
------------------
* Add a ROS-independent, validated online STL monitor core.
* Support per-variable topics, message types, and dotted numeric field paths.
* Add file-based specifications, sampling units, and configurable QoS.
* Correct the CMake/Python/rosidl package layout and executable installation.
* Vendor RTAMT 0.3.5 and ANTLR 4.7 for reproducible Bloom builds.
* Keep ROS entry points isolated from incompatible Python environments and
  update the vendored ANTLR imports for current Python versions.
* Add unit, launch, lint, and end-to-end ROS tests plus a sine-wave example.
* Test Humble, Jazzy, Kilted, and Lyrical in continuous integration.
* Contributors: Immanuel Ampomah Mensah
