import pyglet
from pyglet.gl import *


def get_tex(file):
    tex = pyglet.image.load(file).texture
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    return pyglet.graphics.TextureGroup(tex)

class MyWindow(pyglet.window.Window):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.projection = pyglet.window.Projection3D()
        self.main_batch = pyglet.graphics.Batch()
        self.model = pyglet.model.load("../Aeroshell-Body.obj", batch=self.main_batch)

        self.zoom = 1
        self.center = []

        glEnable(GL_MULTISAMPLE_ARB)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)

    def on_draw(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        glViewport(0, 0, 600, 600)
        glMatrixMode(gl.GL_PROJECTION)
        glLoadIdentity()
        glOrtho(-600, 600, -600, 600, -600, 600)
        glMatrixMode(gl.GL_MODELVIEW)

        self.main_batch.draw()


    def on_mouse_drag(self,x, y, dx, dy, buttons, modifiers):
        glRotatef(1, dx, dy, 0)


    def on_resize(self, width, height):
        glViewport(0, 0, width, height)
    

def get_next_state(dt):
    pass



if __name__ == '__main__':

    config = Config(sample_buffers=1, samples=8)
    window = MyWindow(height=600, width=600, config=config, resizable=False, vsync=False)
    pyglet.clock.schedule_interval(get_next_state, 0.5)
    pyglet.app.run()
    
