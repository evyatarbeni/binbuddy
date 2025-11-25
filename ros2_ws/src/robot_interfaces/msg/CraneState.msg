# Crane position and status
float32 position_percent    # 0-100%
bool homed                  # Has crane been homed?
bool at_lower_limit         # Lower limit switch triggered
bool at_upper_limit         # Upper limit switch triggered
bool moving                 # Currently in motion
string current_preset       # "STOW", "PICK", "PLACE", "CUSTOM"
