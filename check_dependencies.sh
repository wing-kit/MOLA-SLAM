#!/bin/bash
# Dependency Checker for MOLA-SLAM Workspace
# Checks if all required dependencies are installed

echo "=========================================="
echo "MOLA-SLAM Dependency Checker"
echo "=========================================="
echo ""

MISSING=0

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced! Run: source /opt/ros/humble/setup.bash"
    MISSING=$((MISSING + 1))
else
    echo "✅ ROS2 $ROS_DISTRO sourced"
fi

# System libraries to check
SYS_LIBS=(
    "build-essential"
    "cmake"
    "pkg-config"
    "python3-colcon-common-extensions"
    "libwxgtk3.0-gtk3-dev"
    "libglut-dev"
    "libopencv-dev"
    "libeigen3-dev"
    "libsuitesparse-dev"
    "libassimp-dev"
    "liboctomap-dev"
    "libtbb-dev"
    "libgtsam-dev"
    "libyaml-cpp-dev"
    "libcli11-dev"
    "libboost-serialization-dev"
    "libboost-system-dev"
    "python3-matplotlib"
    "python3-numpy"
    "python3-tk"
)

echo ""
echo "Checking system libraries..."
for lib in "${SYS_LIBS[@]}"; do
    if dpkg -l | grep -q "^ii.*$lib"; then
        echo "  ✅ $lib"
    else
        echo "  ❌ $lib - NOT INSTALLED"
        MISSING=$((MISSING + 1))
    fi
done

# ROS2 packages to check
ROS_PKGS=(
    "ros-humble-ament-cmake"
    "ros-humble-rclcpp"
    "ros-humble-rclpy"
    "ros-humble-geometry-msgs"
    "ros-humble-sensor-msgs"
    "ros-humble-nav-msgs"
    "ros-humble-tf2"
    "ros-humble-tf2-ros"
    "ros-humble-cv-bridge"
    "ros-humble-rviz2"
)

echo ""
echo "Checking ROS2 packages..."
for pkg in "${ROS_PKGS[@]}"; do
    if dpkg -l | grep -q "^ii.*$pkg"; then
        echo "  ✅ $pkg"
    else
        echo "  ❌ $pkg - NOT INSTALLED"
        MISSING=$((MISSING + 1))
    fi
done

# Check for conflicting MRPT apt packages
echo ""
echo "Checking for conflicting MRPT packages..."
MRPT_CONFLICT=0
if dpkg -l | grep -q "^ii.*ros-humble-mrpt-lib"; then
    echo "  ⚠️  WARNING: ros-humble-mrpt-* packages found!"
    echo "     These will conflict with source build."
    echo "     Remove with: sudo apt remove ros-humble-mrpt-*"
    MRPT_CONFLICT=1
else
    echo "  ✅ No conflicting MRPT packages"
fi

# Summary
echo ""
echo "=========================================="
if [ $MISSING -eq 0 ] && [ $MRPT_CONFLICT -eq 0 ]; then
    echo "✅ ALL DEPENDENCIES SATISFIED!"
    echo "You can proceed with: colcon build"
elif [ $MISSING -gt 0 ]; then
    echo "❌ MISSING $MISSING DEPENDENCIES"
    echo "Run the installation command from README.md"
fi

if [ $MRPT_CONFLICT -eq 1 ]; then
    echo "⚠️  CONFLICT DETECTED"
    echo "Remove apt MRPT packages before building"
fi
echo "=========================================="
