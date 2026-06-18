"""Make the driver package and the GUI modules importable from the tests."""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "driver", "src"))
sys.path.insert(0, os.path.join(_ROOT, "gui"))
