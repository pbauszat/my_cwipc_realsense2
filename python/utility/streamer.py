"""
A Pointcloud streamer class.
"""
import pathlib

from typing import Generator

import _cwipc_realsense2
from cwipc.util import cwipc_pointcloud_wrapper as Pointcloud
from cwipc.util import cwipc_activesource_wrapper as ActiveSource


class Streamer:
    """
    Pointcloud streamer.
    """

    def __init__(self, config_file: pathlib.Path) -> None:
        """
        Init.
        """

        # Create a grabber and start it
        grabber: ActiveSource = _cwipc_realsense2.cwipc_realsense2(str(config_file))
        started = grabber.start()
        if not started:
            raise RuntimeError("Grabber not started")
        if not grabber.available(True):
            raise RuntimeError("Grabber must be available")

        # Store internals
        self._grabber: ActiveSource = grabber

    def generator(self) -> Generator[Pointcloud, None, None]:
        """
        Yields a stream of point clouds.
        """

        while True:
            pointcloud = self._grabber.get()
            if not pointcloud:
                raise RuntimeError("Must have a point cloud")

            yield pointcloud
