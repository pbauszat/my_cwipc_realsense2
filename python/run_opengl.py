"""
Displays the cwipc point cloud using OpenGL.
"""
import pathlib
import OpenGL.GL as gl
import OpenGL.GL.shaders as gl_shaders

from ctypes import c_void_p
from typing import ClassVar, Generator

from utility.window import OpenGLWindow
from utility.streamer import Streamer, Pointcloud


# ====================================================================================================================================================
# Application
# ====================================================================================================================================================


class Application(OpenGLWindow):
    """
    My application.
    """

    VERTEX_SHADER: ClassVar[str] = """
        #version 430 core
        
        layout(location=0) in vec3 position;
        layout(location=1) in uvec3 input_color;
        layout(location=2) in uint tile;
        
        out vec3 color;

        void main() {
            gl_Position = vec4(position, 1.0);
            color = vec3(input_color) / 255.0;
        }
    """

    FRAGMENT_SHADER: ClassVar[str] = """
        #version 430 core
        
        in vec3 color;
        out vec4 output_color;

        void main() {
            output_color = vec4(color, 1.0);
        }
    """

    def __init__(self, streamer: Streamer) -> None:
        """
        Init.
        """

        super().__init__((640, 480), "Application")

        self._generator: Generator[Pointcloud, None, None] = streamer.generator()
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

        pass

    def render(self):
        """
        Renders something.
        """

        point_count = self._update_point_cloud()

        gl.glViewport(0, 0, self.width, self.height)
        gl.glClearColor(0.0, 0.0, 0.2, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        gl.glUseProgram(self._shader_program)
        gl.glBindVertexArray(self._vertex_array)
        gl.glDrawArrays(gl.GL_POINTS, 0, point_count)

        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

    def _update_point_cloud(self) -> int:
        """
        Update point cloud.
        """

        # Get pointcloud data.
        # Each point is a tuple that follows the CWIPC point structure:
        #   x - float32
        #   y - float32
        #   z - float32
        #   r - unsigned byte
        #   g - unsigned byte
        #   b - unsigned byte
        #   tile - unsigned byte
        # The total size of the structure is 16 bytes.
        pointcloud = next(self._generator)
        point_data = pointcloud.get_numpy_array()
        vertex_stride = 16

        # Upload the data to the vertex buffer
        if len(point_data) > 0:
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self._vertex_buffer)
            gl.glBufferData(gl.GL_ARRAY_BUFFER, point_data, gl.GL_STATIC_DRAW)

            gl.glBindVertexArray(self._vertex_array)
            gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
            gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, vertex_stride, None)
            gl.glEnableVertexAttribArray(0)
            gl.glVertexAttribIPointer(1, 3, gl.GL_UNSIGNED_BYTE, vertex_stride, c_void_p(12))
            gl.glEnableVertexAttribArray(1)
            gl.glVertexAttribIPointer(2, 1, gl.GL_UNSIGNED_BYTE, vertex_stride, c_void_p(15))
            gl.glEnableVertexAttribArray(2)

            gl.glBindVertexArray(0)
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

        return len(point_data)


# ====================================================================================================================================================
# Main
# ====================================================================================================================================================


def main():
    config_file = pathlib.Path("C:\\Users\\Pablo\\Documents\\Projects\\my_cwipc\\cameraconfig.json")
    streamer = Streamer(config_file)
    application = Application(streamer)
    application.run()


if __name__ == "__main__":
    main()
