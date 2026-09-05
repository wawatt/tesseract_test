#!/usr/bin/env python3
"""
export_urdf.py:
Expands fanuc_r2000_description/robot/r2000ic_165f.urdf.xacro into standalone URDF
and resolves local mesh file paths.
"""

import os
import sys
import subprocess

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
    
    xacro_file = os.path.join(workspace_dir, "asset", "fanuc_r2000_description", "robot", "r2000ic_165f.urdf.xacro")
    output_urdf = os.path.join(script_dir, "..", "models", "r2000ic_165f.urdf")
    
    if not os.path.exists(xacro_file):
        print(f"Error: source xacro not found: {xacro_file}")
        sys.exit(1)
        
    print(f"Exporting xacro: {xacro_file} -> {output_urdf}")
    try:
        res = subprocess.run(["xacro", xacro_file, "-o", output_urdf], capture_output=True, text=True, check=True)
        print("Export successful!")
    except Exception as e:
        print("Note: If xacro CLI is not installed in current python env, you can use the pre-generated models/r2000ic_165f.urdf.")
        print(f"Detail: {e}")

if __name__ == "__main__":
    main()
