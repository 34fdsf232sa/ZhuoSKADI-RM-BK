#!/usr/bin/env python3
"""
CSV Waypoint Loader — COD-style waypoint file I/O.

Format (same as cod_bringup/wps/*.csv):
  id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command

Usage:
  from mapless_nav_v2.waypoint_loader import load_waypoints, save_waypoints
  wps = load_waypoints('patrol_front.csv')
"""

import csv
import os
from geometry_msgs.msg import PoseStamped


def load_waypoints(filepath: str, frame_id: str = 'odom') -> list[PoseStamped]:
    """
    Load waypoints from a CSV file.

    CSV columns: id, pose_x, pose_y, pose_z, rot_x, rot_y, rot_z, rot_w, command

    Returns:
        List of PoseStamped in the given frame_id.
    """
    waypoints = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            wp = PoseStamped()
            wp.header.frame_id = frame_id
            wp.pose.position.x = float(row['pose_x'])
            wp.pose.position.y = float(row['pose_y'])
            wp.pose.position.z = float(row.get('pose_z', 0.0))
            wp.pose.orientation.x = float(row.get('rot_x', 0.0))
            wp.pose.orientation.y = float(row.get('rot_y', 0.0))
            wp.pose.orientation.z = float(row.get('rot_z', 0.0))
            wp.pose.orientation.w = float(row.get('rot_w', 1.0))
            waypoints.append(wp)
    return waypoints


def save_waypoints(filepath: str, waypoints: list[PoseStamped],
                   commands: list[str] = None):
    """
    Save waypoints to a CSV file.

    Args:
        filepath: Output CSV path.
        waypoints: List of PoseStamped.
        commands: Optional list of command strings (e.g. 'patrol', 'start', 'end').
    """
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['id', 'pose_x', 'pose_y', 'pose_z',
                         'rot_x', 'rot_y', 'rot_z', 'rot_w', 'command'])
        for i, wp in enumerate(waypoints):
            cmd = commands[i] if commands and i < len(commands) else 'patrol'
            writer.writerow([
                i,
                f'{wp.pose.position.x:.4f}',
                f'{wp.pose.position.y:.4f}',
                f'{wp.pose.position.z:.4f}',
                f'{wp.pose.orientation.x:.4f}',
                f'{wp.pose.orientation.y:.4f}',
                f'{wp.pose.orientation.z:.4f}',
                f'{wp.pose.orientation.w:.4f}',
                cmd,
            ])


def get_package_wps_path(package_name: str = 'mapless_nav_v2') -> str:
    """Get the wps/ directory path for a package."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package_name), 'wps')
