#!/usr/bin/env python3
"""Inicia el mapa fijo y la localización oficial de Nav2."""

import argparse

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


NAV2_NAMESPACE = "nav2"
MANAGED_NODES = ["map_server", "amcl"]
COMMON_REMAPPINGS = [("/tf", "/tf"), ("/tf_static", "/tf_static")]


def build_launch_description(params_file):
    """Mantiene juntos el mapa y quien corrige la posición sobre ese mapa."""
    nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            namespace=NAV2_NAMESPACE,
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[params_file],
            remappings=COMMON_REMAPPINGS + [("map", "/map")],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            namespace=NAV2_NAMESPACE,
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[params_file],
            remappings=COMMON_REMAPPINGS + [("amcl_pose", "/amcl_pose")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            namespace=NAV2_NAMESPACE,
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "autostart": True,
                    "node_names": MANAGED_NODES,
                    "bond_timeout": 4.0,
                }
            ],
        ),
    ]
    return LaunchDescription(nodes)


def main():
    parser = argparse.ArgumentParser(
        description="Inicia un mapa fijo y localiza al G1 con AMCL"
    )
    parser.add_argument("--params-file", required=True)
    args = parser.parse_args()
    service = LaunchService()
    service.include_launch_description(
        build_launch_description(args.params_file)
    )
    return service.run()


if __name__ == "__main__":
    raise SystemExit(main())
