import numpy as np
import pybullet as p
import pytest
import vision
import train_vision as T
from perception import camera


@pytest.mark.skipif(not vision.available(), reason="ultralytics / trained model not installed")
def test_yolo_finds_a_person_and_nothing_on_empty_ground():
    cl = p.connect(p.DIRECT)
    T.box(cl, [0, 0, -0.01], [3, 3, 0.01], [0.5, 0.48, 0.42, 1])
    shot = lambda: vision.detect(camera(cl, [0, -0.3, 1.0], [0, 0, 0], 320, 320)[0], conf=0.5)
    empty = shot()
    T.spawn(cl, "person", 0, 0, np.random.default_rng(3))
    seen = shot()
    p.disconnect(cl)
    assert not empty and seen and seen[0][0] == "person"


import pilot_rl


@pytest.mark.skipif(not pilot_rl.available(), reason="trained pilot not installed")
def test_learned_pilot_reaches_waypoints_on_unseen_courses():
    r = pilot_rl.evaluate(n=300, seed=7)
    assert r["goal"] > 0.85 and r["crash"] < 0.08
