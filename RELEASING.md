# Releasing rtamt4ros2

This package follows the standard ROS 2 Bloom release process. Runtime
dependencies are represented by rosdep keys, and the PyPI-only RTAMT and ANTLR
versions are vendored so release-farm builds do not access the network.

## Prepare the source release

1. Run the full build and test suite for every supported ROS distribution.
2. Confirm that `package.xml` and `CHANGELOG.rst` contain the release version.
3. Confirm `rosdep check --from-paths . --ignore-src` succeeds.
4. Run a local Bloom generation check for the target platform.
5. Commit the release changes and create an annotated version tag:

   ```bash
   git tag -a 0.2.0 -m "Release 0.2.0"
   git push origin main
   git push origin 0.2.0
   ```

## First ROS distribution release

The ROS release team must first create
`https://github.com/ros2-gbp/rtamt4ros2-release`. Then run:

```bash
bloom-release --new-track --rosdistro humble --track humble rtamt4ros2
```

Review and merge the generated rosdistro pull request. Repeat with a distinct
track for each additional supported distribution. Later releases use:

```bash
bloom-release --rosdistro humble --track humble rtamt4ros2
```

The package becomes installable with
`apt install ros-${ROS_DISTRO}-rtamt4ros2` after the rosdistro pull request is
merged and the ROS build farm completes the binary jobs.
