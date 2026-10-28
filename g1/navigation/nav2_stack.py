#!/usr/bin/env python3
"""Inicia el subconjunto oficial de Nav2 que usa la demo del G1."""

import argparse

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


NAV2_NAMESPACE = "nav2"
MANAGED_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "bt_navigator",
]
COMMON_REMAPPINGS = [("/tf", "/tf"), ("/tf_static", "/tf_static")]


def managed_node(package, executable, name, params_file, remappings=None):
    """Construye un servidor oficial con reinicio visible ante una caída."""
    return Node(
        package=package,
        executable=executable,
        name=name,
        namespace=NAV2_NAMESPACE,
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file],
        remappings=COMMON_REMAPPINGS + (remappings or []),
    )


def build_launch_description(params_file):
    """Mantiene sólo las piezas necesarias; no arranca docking ni rutas."""
    nodes = [
        managed_node(
            "nav2_controller",
            "controller_server",
            "controller_server",
            params_file,
            [("cmd_vel", "cmd_vel_nav")],
        ),
        managed_node(
            "nav2_planner",
            "planner_server",
            "planner_server",
            params_file,
        ),
        managed_node(
            "nav2_behaviors",
            "behavior_server",
            "behavior_server",
            params_file,
            [("cmd_vel", "cmd_vel_nav")],
        ),
        managed_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            "velocity_smoother",
            params_file,
            [
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "/g1/cmd_vel/navigation"),
            ],
        ),
        managed_node(
            "nav2_bt_navigator",
            "bt_navigator",
            "bt_navigator",
            params_file,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
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
        description="Inicia Nav2 conectado a la autoridad del G1"
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

