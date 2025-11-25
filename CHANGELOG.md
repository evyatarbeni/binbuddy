# Changelog

## [1.0.1] - 2024-11-21

### Fixed
- Arduino auto-detection for /dev/ttyACM1
- NumPy compatibility issue (downgraded to <2 for cv_bridge)
- LiDAR driver pthread compilation error
- Bootstrap workspace path handling
- Deploy script robustness

### Added
- Arduino port detection script
- Comprehensive testing documentation
- Improved error messages
- Better launch file configuration

### Changed
- Updated README with known issues
- Enhanced deployment script with better checks
- Improved install_dependencies.sh reliability

## [1.0.0] - 2024-11-21

### Initial Release
- Complete ROS 2 Jazzy workspace
- Differential drive motor control
- Crane control with presets
- Triple camera support (USB + 2x Pi cameras)
- LiDAR integration
- Natural language control (ChatGPT)
- Keyboard teleop
- Navigation and SLAM support
