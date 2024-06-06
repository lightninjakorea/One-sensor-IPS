import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
import os
import PIL.Image as pilimg
import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import math
import pylab
from afasf import pred

xml_path = 'env.xml'
print_camera_config = 0

button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

_overlay = {}


def find_a(x_list, y_list):
    n = len(x_list)
    sx, sx2, sy, sxy = sum(x_list), 0, sum(y_list), 0
    for i in range(n):
        sx2 += x_list[i] ** 2
        sxy += x_list[i] * y_list[i]
    if (n * sxy > sx * sy) ^ (n * sx2 > sx): return True  # True 일 때 증가하는 것임
    else : return False


def add_overlay(gridpos, text1, text2):
    if gridpos not in _overlay:
        _overlay[gridpos] = ["", ""]
    _overlay[gridpos][0] += text1 + "\n"
    _overlay[gridpos][1] += text2 + "\n"


def create_overlay(model, data):
    topleft = mj.mjtGridPos.mjGRID_TOPLEFT
    topright = mj.mjtGridPos.mjGRID_TOPRIGHT
    bottomleft = mj.mjtGridPos.mjGRID_BOTTOMLEFT
    bottomright = mj.mjtGridPos.mjGRID_BOTTOMRIGHT


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_DOWN:
        data.qvel[6] = 3

    if act == glfw.PRESS and key == glfw.KEY_UP:
        data.qvel[6] = -3

    if act == glfw.PRESS and key == glfw.KEY_RIGHT:
        data.qvel[5] = -3

    if act == glfw.PRESS and key == glfw.KEY_LEFT:
        data.qvel[5] = 3
    if act == glfw.PRESS and key == glfw.KEY_SPACE:
        data.qvel[5] = 0
        data.qvel[6] = 0


def init_controller(model, data):
    pass


def controller(model, data):
    pass


def mouse_button(window, button, act, mods):
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    glfw.get_cursor_pos(window)


def mouse_move(window, xpos, ypos):
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    if (not button_left) and (not button_middle) and (not button_right):
        return

    width, height = glfw.get_window_size(window)

    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx / height,
                      dy / height, scene, cam)


def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)


dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
cam = mj.MjvCamera()
opt = mj.MjvOption()

glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

cam.azimuth = 84.69038085937497;
cam.elevation = -16.31253662109375;
cam.distance = 7.5876800784056755
cam.lookat = np.array([1, 0.0, 1.0])
init_controller(model, data)
mj.set_mjcb_control(controller)

N = 500
i = 0
time = 0
dt = 0.001
simend = 10000

x, y = [], []

distanceData = []
positionData = []
enable = True
while not glfw.window_should_close(window):
    time_prev = time
    enable_first=1
    while (time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)
        data.qvel[0] = 5
        distanceData.append(
            ((data.sensordata[0] - data.sensordata[3]) ** 2 + (data.sensordata[1] - data.sensordata[4]) ** 2) ** 0.5)
        positionData.append((data.qpos[0] + np.pi / 2))
        time += dt
        #angle = positionData[distanceData.index(max(distanceData))]
        restart=3
        
        if (len(distanceData) > 5+restart):
            if enable_first:
                check = find_a(positionData, distanceData)
                #print(check)
                enable_first = 0
            #print(check)
            if check != find_a(positionData, distanceData) : #증가 감소 경향성 바뀜 -> 극대 or 극소
                check = find_a(positionData, distanceData)
                #print("change")
                #print(positionData, distanceData)
                func_pred, max_postion, max_distance=pred(1,1,1,positionData, distanceData, check)
                #print(max_postion, max_distance)
                print(abs((max_distance)*np.cos(max_postion)), abs((max_distance)*np.sin(max_postion)))
                print(abs(data.sensordata[3]), abs(data.sensordata[4]))
            for _ in range(restart) :
                distanceData.pop(0)
                positionData.pop(0)
            
    # print('d:',(data.qpos[0] + np.pi / 2) % (2 * np.pi))
    # print('t:',time)

    i += 1

    if (data.time >= simend):
        break

    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    if (print_camera_config == 1):
        print('cam.azimuth =', cam.azimuth, ';', 'cam.elevation =', cam.elevation, ';', 'cam.distance = ', cam.distance)
        print('cam.lookat =np.array([', cam.lookat[0], ',', cam.lookat[1], ',', cam.lookat[2], '])')
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()