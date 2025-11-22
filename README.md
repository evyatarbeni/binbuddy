# Changelog

All notable changes to BinBuddy will be documented in this file.

## [1.0.1] - 2024-11-21

### Fixed
- Arduino auto-detection for `/dev/ttyACM1` (Raspberry Pi 5 default)
- NumPy compatibility issue with cv_bridge (downgraded to <2.0)
- LiDAR driver pthread compilation error (auto-patched during deployment)
- Bootstrap workspace path handling (now correctly uses `~/binbuddy_ws`)
- Deploy script robustness and error handling
- Missing ROS 2 build dependencies (ament_cmake, rosidl_default_generators)

### Added
- Arduino port detection script (`scripts/detect_arduino.sh`)
- Comprehensive testing documentation (`docs/TESTING.md`)
- Improved error messages and user guidance in all scripts
- Better launch file configuration with proper parameter handling
- Automatic pthread.h patch for LiDAR driver
- CHANGELOG.md for version tracking

### Changed
- Updated README.md with known issues and fixes
- Enhanced deployment script with better pre-flight checks
- Improved `install_dependencies.sh` with explicit package list
- Better documentation structure and clarity

## [1.0.0] - 2024-11-20

### Added
- Initial release of BinBuddy autonomous robot system
- Complete ROS 2 Jazzy workspace structure
- Differential drive motor control with odometry
- Crane control system with limit switches and presets
- Triple camera support (1x USB + 2x Raspberry Pi cameras)
- LDROBOT STL-19P LiDAR integration
- Natural language control via ChatGPT API
- Keyboard teleop interface
- Nav2 navigation stack integration
- SLAM Toolbox mapping capabilities
- One-command deployment script
- Comprehensive bootstrap system for package generation
- Safety systems (movement enable/disable, emergency stop)
- System health monitoring
- URDF robot description
- RViz configuration
- udev rules for device management

### Hardware Support
- Raspberry Pi 5 (4GB+ RAM)
- Ubuntu 24.04 Server (64-bit ARM)
- Arduino Mega 2560
- DC motor differential drive
- Linear actuator crane

### Documentation
- Quick start guide
- Hardware setup guide
- API reference
- Troubleshooting guide

## [Unreleased]

### Planned Features
- Web dashboard for remote monitoring
- Autonomous bin detection using computer vision
- Path planning improvements
- Battery monitoring and low-power warnings
- Multi-robot coordination
- Improved natural language understanding
- Mobile app for control

---

## Version History Notes

### Version Format
- MAJOR.MINOR.PATCH
- MAJOR: Incompatible API changes
- MINOR: New functionality (backwards compatible)
- PATCH: Bug fixes (backwards compatible)

### Support
- Current version: 1.0.1
- Supported platforms: Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy
- Python version: 3.12
- ROS 2 version: Jazzy Jalisco
