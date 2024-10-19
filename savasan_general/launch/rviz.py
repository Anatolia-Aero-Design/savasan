#!/usr/bin/env python

import subprocess

def run_terminal_command():
    # Properly escape double quotes within the command string
    command = 'gnome-terminal --tab --title="Rviz" -- bash -c "rviz -d $(rospack find rviz_satellite)/launch/demo.rviz; exec bash"'
    subprocess.call(command, shell=True)

if __name__ == "__main__":
    run_terminal_command()
