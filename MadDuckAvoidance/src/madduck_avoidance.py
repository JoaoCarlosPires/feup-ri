#!/usr/bin/env python

from PIL import Image
import argparse
import sys
import numpy as np
import pyglet
from pyglet.window import key
from gym_duckietown.envs import DuckietownEnv

from detect_duckie import findObjects

parser = argparse.ArgumentParser()
parser.add_argument("--map-name", default="custom_map")
args = parser.parse_args()

env = DuckietownEnv(
    seed=1,
    map_name=args.map_name,
    frame_skip=1,
    distortion=False,
    camera_rand=False,
)

env.reset()
env.render()
start = 0

@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
    """
    This handler processes keyboard commands that
    control the simulation
    """
    global start

    if symbol == key.BACKSPACE:
        print("RESET")
        env.reset()
        env.render()
        start = 0
    elif symbol == key.ESCAPE:
        env.close()
        sys.exit(0)
    elif symbol == key.SPACE:
        print("START")
        start = 1


# Register a keyboard handler
key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)
detection = False
velocity = 0
angular = 0
counter = 0
detectionCounter = 0

def decide_action(ducks):
    global detection, velocity, angular
    
    for duck in ducks:
        if duck[3] >= 40:
            if duck[0] < 320 and duck[0] > 200: # Duckie is at the left side
                velocity = 0.2
                angular = -1.2
                detection = True
            elif duck[0] >= 320 and duck[0] < 440: # Duckie is at the right side
                velocity = 0.2
                angular = 1.2
                detection = True

def update(dt):
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """
    wheel_distance = 0.102
    min_rad = 0.08
    action = np.array([0.0, 0.0])

    global start, detection, velocity, angular, counter, detectionCounter
    counter += 1
    if not detection:
        if start == 0:
            if key_handler[key.UP]:
                action += np.array([0.44, 0])
            if key_handler[key.DOWN]:
                action -= np.array([0.44, 0])
            if key_handler[key.LEFT]:
                action += np.array([0, 1])
            if key_handler[key.RIGHT]:
                action -= np.array([0, 1])
            if key_handler[key.SPACE]:
                action = np.array([0, 0])
        else:
            action[0] = 0.2
            action[1] = 0.0
    else:
        if detectionCounter == 200:
            detection = False
            detectionCounter = 0
        else:
            if detectionCounter > 50 and detectionCounter < 150:
                action[1] = -angular  
            else:
                action[1] = angular  
            
            action[0] = velocity
            detectionCounter += 1

    obs, reward, done, info = env.step(action)

    im = Image.fromarray(obs)

    if counter % 10 == 0 and not detection:
        ducks = findObjects(obs)
        if (len(ducks) != 0 ):
            decide_action(ducks)

    if key_handler[key.RETURN]:
        im.save("screen.png")

    env.render()


pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

# Enter main event loop
pyglet.app.run()

env.close()
