"""
Displays the RGB-D stream from a camera using the cwipc API.
"""
import cv2
import numpy as np
import _cwipc_realsense2

from typing import Optional

from cwipc.util import cwipc_pointcloud_wrapper as Pointcloud
from cwipc.util import cwipc_activesource_wrapper as ActiveSource

# ====================================================================================================================================================
# Frames
# ====================================================================================================================================================


def get_frame(grabber: ActiveSource) -> tuple[np.ndarray, int]:
    """
    Returns a timedstamped frame.
    """

    # Read point cloud from the grabber
    pointcloud: Optional[Pointcloud] = grabber.get()
    assert pointcloud, "Must have a point cloud"

    # Get the points
    points = pointcloud.get_points()
    # print(points)
    print(f"Points {len(points)}")

    # Convert point cloud to image
    metadata = pointcloud.access_metadata()
    assert metadata, "Must have meta data"
    # print(f"{metadata=}")
    # print(f"Metadata count: {metadata.count()}")

    # for i in range(metadata.count()):
    #    print(i, metadata.name(i), metadata.data(i), type(metadata.data(i)))

    images = metadata.get_all_images("rgb.")
    # assert images, "Must have images"
    if not images:
        image = np.zeros((1, 1, 3))
    else:
        assert len(images) == 1, "Only supports single camera image"
        image = list(images.values())[0]

    assert len(image.shape) == 3, "Image must be of proper shape"

    timestamp = pointcloud.timestamp()
    return image, timestamp


# ====================================================================================================================================================
# Main
# ====================================================================================================================================================


def main():
    # Create a grabber and start it
    # config_file = "auto"
    config_file = "C:\\Users\\Pablo\\Documents\\Projects\\my_cwipc\\cameraconfig.json"
    grabber: ActiveSource = _cwipc_realsense2.cwipc_realsense2(config_file)
    started = grabber.start()
    assert started, "Grabber not started"
    assert grabber.available(True), "Grabber must be available"

    # Select meta-data
    grabber.request_metadata("rgb")

    # Create window
    window_name = "Stream"
    window = cv2.namedWindow(window_name)

    # Process camera stream in a while loop
    paused = False
    while True:
        key: int = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == 32:
            paused = not paused

        if not paused:
            image, timestamp = get_frame(grabber)
            combined_image = image  # cv2.hconcat([image, image])
            height, width, _ = image.shape
            title = f"Stream ({timestamp})"
            cv2.imshow(window_name, combined_image)
            cv2.resizeWindow(window_name, width, height)
            cv2.setWindowTitle(window_name, title)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
