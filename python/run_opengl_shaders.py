"""
Displays the cwipc point cloud using OpenGL.
"""
import abc
import glfw
import numpy as np
import OpenGL.GL as gl
import OpenGL.GL.shaders as gl_shaders

from typing import Any, ClassVar

# ====================================================================================================================================================
# Base OpenGL Window class
# ====================================================================================================================================================


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

# ====================================================================================================================================================
# Application
# ====================================================================================================================================================


class Application(OpenGLWindow):
    """
    My application.
    """

    VERTEX_SHADER: ClassVar[str] = """
        #version 330 core
        
        layout(location=0) in vec3 position;

        void main() {
            gl_Position = vec4(position, 1.0);
        }
    """

    FRAGMENT_SHADER: ClassVar[str] = """
        #version 330 core
        
        out vec3 color;

        void main() {
            color = vec3(0.0, 1.0, 0.0);
        }
    """

    def __init__(self) -> None:
        """
        Init.
        """

        super().__init__((640, 480), "Application")

        self._vertex_array: gl.GLuint = gl.glGenVertexArrays(1)
        self._vertex_buffer: gl.GLuint = gl.glGenBuffers(1)
        self._shader_program: gl.GLuint = gl_shaders.compileProgram(
            gl_shaders.compileShader(Application.VERTEX_SHADER, gl.GL_VERTEX_SHADER),
            gl_shaders.compileShader(Application.FRAGMENT_SHADER, gl.GL_FRAGMENT_SHADER),
        )

    def setup(self):
        """
        Setup rendering.
        """

        vertices = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]
        vertices = np.array(vertices, dtype=np.float32).flatten()
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self._vertex_buffer)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, vertices, gl.GL_STATIC_DRAW)

        gl.glBindVertexArray(self._vertex_array)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, None)
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)

        gl.glBindVertexArray(0)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def render(self):
        """
        Renders something.
        """

        gl.glViewport(0, 0, self.width, self.height)
        gl.glClearColor(1.0, 0.0, 1.0, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        gl.glUseProgram(self._shader_program)
        gl.glBindVertexArray(self._vertex_array)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 3)

        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

# ====================================================================================================================================================
# Main
# ====================================================================================================================================================


def main():
    application = Application()
    application.run()


if __name__ == "__main__":
    main()
