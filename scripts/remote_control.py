#!/usr/bin/env python3
"""
Direct executable launcher for Wobble Remote Control.
Usage:
    ./scripts/remote_control.py        (GUI mode)
    ./scripts/remote_control.py --cli  (Terminal CLI mode)
"""

import os
import sys

# Ensure ROS 2 package path is available
workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.join(workspace_dir, 'src', 'wobble_control')
sys.path.insert(0, src_dir)

# Set XCB platform and library preload for graphics
if 'QT_QPA_PLATFORM' not in os.environ:
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

from wobble_control.remote_control import main

if __name__ == '__main__':
    main()
