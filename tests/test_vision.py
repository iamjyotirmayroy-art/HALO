import os
import cv2
import numpy as np
import pybullet as p
import pytest
import humans
import perception as P
import vision


def test_ai_check_never_discards_and_maps_confidence_to_status():
    empty = P.ai_check([], 35.0)
    assert empty["status"].startswith("POSSIBLE") and empty["confidence"] == 0 and empty["detections"] == []
    assert P.ai_check([("person", 0.91, (0, 0, 9, 9))], 36.0)["status"] == "CONFIRMED VICTIM"
    likely = P.ai_check([("body part: hand/arm", 0.6, (0, 0, 9, 9)), ("person", 0.3, (0, 0, 9, 9))], 33.0)
    assert likely["status"].startswith("LIKELY") and likely["label"] == "body part: hand/arm" and len(likely["detections"]) == 2
    assert P.ai_check([("person", 0.3, (0, 0, 9, 9))], 30.0)["label"] == "unclear (person?)"


@pytest.fixture
def scene():
    cl = p.connect(p.DIRECT)
    yield cl
    p.disconnect(cl)


def test_every_victim_kind_is_built_and_partial_ones_are_buried(scene):
    rng = np.random.default_rng(0)
    for i, kind in enumerate(humans.KINDS):
        body, debris = humans.build(scene, kind, (i * 2.0, 0), 0.5, rng)
        assert len(body) >= 2
        assert kind == "person" or debris            # hand / foot / head stick out of rubble


@pytest.mark.skipif(not os.path.exists(vision.TUNED), reason="fine-tuned detector not trained (python simulation/train_detector.py)")
def test_tuned_yolov8_finds_each_kind_from_the_drone(scene):
    pytest.importorskip("ultralytics")
    rng = np.random.default_rng(4)
    p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE, physicsClientId=scene),
                      p.createVisualShape(p.GEOM_PLANE, rgbaColor=[0.47, 0.45, 0.41, 1], physicsClientId=scene), physicsClientId=scene)
    found = {}
    for i, kind in enumerate(humans.KINDS):
        humans.build(scene, kind, (i * 3.0, 0), rng.uniform(0, 6), rng)
        rgb, _ = P.camera(scene, [i * 3.0 - 0.3, -0.3, 0.8], [i * 3.0, 0, 0.05], 320, 320, fov=70)
        found[kind] = vision.detect(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    for kind, dets in found.items():
        assert dets and dets[0][0] == vision.NAMES[kind] and dets[0][1] > 0.5, (kind, dets)
