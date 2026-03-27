"""
A OpenGL window base class.
"""
import abc
import glfw

from typing import Any


class OpenGLWindow(abc.ABC):
    """
    Base OpenGL window class.
    """

    def __init__(self, window_size: tuple[int, int] = (640, 480), title: str = "OpenGL Window") -> None:
        """
        Init.
        """

        # Initialize GLFW
        if not glfw.init():
            raise RuntimeError("Could not initialize GLFW.")

        # Create window
        width, height = window_size
        window = glfw.create_window(width, height, title, None, None)
        if not window:
            glfw.terminate()
            raise RuntimeError("Could not create GLFW window.")

        # Make the window's context current
        glfw.make_context_current(window)

        # Store internal
        self._internal_window: Any = window

    def run(self) -> None:
        """
        Runs the main loop.
        """

        # Allow the application to setup before rendering
        self.setup()

        # Loop until the user closes the window
        while not glfw.window_should_close(self._internal_window):
            self.render()
            glfw.swap_buffers(self._internal_window)
            glfw.poll_events()

        glfw.terminate()

    @property
    def width(self) -> int:
        """
        Window width.
        """

        return glfw.get_window_size(self._internal_window)[0]

    @property
    def height(self) -> int:
        """
        Window height.
        """

        return glfw.get_window_size(self._internal_window)[1]

    @abc.abstractmethod
    def setup(self) -> None:
        """
        Abstract setup method (implemented by derived classes).
        """

        pass

    @abc.abstractmethod
    def render(self) -> None:
        """
        Abstract render method (implemented by derived classes).
        """

        pass
