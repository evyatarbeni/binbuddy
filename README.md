cat >> ~/binbuddy/CHANGELOG.md << 'EOF'

## [1.0.2] - 2024-11-24

### Fixed
- **CRITICAL**: Motor control system now working
  - Changed from binary protocol to ASCII protocol
  - Base controller sends correct commands ('F', 'B', 'L', 'R')
  - Added automatic speed initialization on startup
  - Implemented heartbeat to prevent watchdog timeout

### Changed
- Updated `base_controller.py` to use ASCII commands
- Updated `crane_controller.py` to use proportional control
- Set default motor speed to 30 (safe/testing level)

### Tested
- Motors respond to ROS 2 velocity commands
- Movement enable/disable functional
- Emergency stop functional
- Watchdog safety system operational

### Known Issues
- Speed is set to LOW (30) for safety - increase for normal operation
- LiDAR not configured (wrong parameters)
- Camera NumPy compatibility warning
EOF
