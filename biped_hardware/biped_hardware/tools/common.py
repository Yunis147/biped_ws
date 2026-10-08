"""
Shared helpers for the command-line tools.

These tools communicate with the servos directly.

IMPORTANT:
Stop servo_bridge before running calibration tools.
"""

from __future__ import annotations

import argparse

from ..bus_factory import open_bus
from ..config import load_config
from ..servo_array import ServoArray


def add_common_args(p: argparse.ArgumentParser) -> None:
    """
    Add arguments shared by all calibration/servo CLI tools.
    """

    p.add_argument(
        "--port",
        default=None,
        help=(
            "serial port; default is from hardware.yaml. "
            "'auto' discovers both servo boards."
        ),
    )

    p.add_argument(
        "--baud",
        type=int,
        default=None,
        help="baud rate; default is from hardware.yaml",
    )

    p.add_argument(
        "--mock",
        action="store_true",
        help="use simulated servos instead of real hardware",
    )

    p.add_argument(
        "--calibration",
        default=None,
        help=(
            "calibration YAML file; "
            "default is ~/.config/biped/calibration.yaml"
        ),
    )

    p.add_argument(
        "--hardware-config",
        default=None,
        help="path to hardware.yaml",
    )

    p.add_argument(
        "--servos-config",
        default=None,
        help="path to servos.yaml",
    )


def setup(args, bus=None):
    """
    Create the configuration, bus and ServoArray.

    Returns:

        cfg, bus, arr

    In real hardware with port='auto':

        bus = MultiBus
        arr = ServoArray(MultiBus, cfg)

    ServoArray handles routing each servo ID to the
    appropriate physical serial board.
    """

    cfg = load_config(
        args.hardware_config,
        args.servos_config,
        args.calibration,
    )

    bus = bus or open_bus(
        cfg,
        port=args.port,
        mock=args.mock,
        baud=args.baud,
    )

    arr = ServoArray(
        bus,
        cfg,
    )

    return cfg, bus, arr


def ask(prompt: str) -> str:
    """
    Read a response from the user.
    """

    return input(prompt)


def say(text: str = "") -> None:
    """
    Print CLI output.
    """

    print(text)