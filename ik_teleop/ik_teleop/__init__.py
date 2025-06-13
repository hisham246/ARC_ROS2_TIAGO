# ik_teleop/__init__.py

import os
import importlib

# Get the directory of this file
package_dir = os.path.dirname(__file__)

# Dynamically import all classes from all modules in this package
for module_name in os.listdir(package_dir):
    if module_name != "__init__.py":
        # Remove the .py extension to get the module name
        module_name = module_name[:-3]
        # Import the module dynamically
        module = importlib.import_module(f".{module_name}", package="ik_teleop")
        # Get all classes from the module and add them to the current namespace
        for attribute_name in dir(module):
            attribute = getattr(module, attribute_name)
            if isinstance(attribute, type):
                globals()[attribute_name] = attribute

# Clean up namespace (optional)
del os, importlib, module_name, module, attribute_name, attribute, package_dir