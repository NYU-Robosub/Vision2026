"""Real ROS/CvBridge tests with a fake detector; no weights, camera or GPU required."""

import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from chair_detection import node as module


@pytest.fixture
def detector(monkeypatch):
    context = Context()
    rclpy.init(context=context)
    model = Mock()
    model.predict.return_value = [SimpleNamespace(boxes=[object(), object()])]
    monkeypatch.setattr(module, 'load_model', lambda path: model)
    node = module.ChairDetection(context=context)
    node.inference_timer.cancel()
    node.report_timer.cancel()
    yield node, model
    node.destroy_node()
    context.shutdown()


def frame(node, value=0):
    return node.bridge.cv2_to_imgmsg(np.full((8, 8, 3), value, dtype=np.uint8), 'bgr8')


def test_latest_frame_cuda_chair_filter_and_no_saving(detector):
    node, model = detector
    node.receive(frame(node, 1))
    node.receive(frame(node, 2))
    node.infer()
    options = model.predict.call_args.kwargs
    assert np.all(options['source'] == 2)
    assert options['device'] == 0
    assert options['classes'] == [56]
    assert options['conf'] == 0.5
    for key in ('save', 'save_txt', 'save_crop', 'save_conf', 'show', 'visualize'):
        assert options[key] is False
    node.infer()
    assert model.predict.call_count == 1  # Never reprocess an old frame.


def test_count_published_including_zero(detector):
    node, model = detector
    received = []
    subscriber = node.create_subscription(Int32, '/vision/chair_count',
                                          lambda msg: received.append(msg.data), 10)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        for expected in (2, 0):
            model.predict.return_value = [SimpleNamespace(boxes=[None] * expected)]
            node.receive(frame(node))
            node.infer()
            deadline = time.monotonic() + 3
            while len(received) < (1 if expected == 2 else 2) and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
        assert received == [2, 0]
    finally:
        executor.shutdown()
        node.destroy_subscription(subscriber)


def test_failure_does_not_publish_false_zero_and_recovers(detector):
    node, model = detector
    publisher = Mock()
    original = node.publisher
    node.publisher = publisher
    try:
        model.predict.side_effect = RuntimeError('GPU failure')
        node.receive(frame(node))
        node.infer()
        publisher.publish.assert_not_called()
        model.predict.side_effect = None
        node.receive(frame(node))
        node.infer()
        assert publisher.publish.call_args.args[0].data == 2
    finally:
        node.publisher = original


def test_receives_new_frames_during_slow_inference(detector):
    node, model = detector
    entered, release = threading.Event(), threading.Event()

    def slow_predict(**kwargs):
        entered.set()
        assert release.wait(5)
        return [SimpleNamespace(boxes=[])]

    model.predict.side_effect = slow_predict
    source = Node('test_camera', context=node.context)
    publisher = source.create_publisher(Image, node.get_parameter('image_topic').value, 1)
    executor = MultiThreadedExecutor(num_threads=2, context=node.context)
    executor.add_node(node)
    executor.add_node(source)
    node.receive(frame(node, 1))
    node.inference_timer.reset()
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        assert entered.wait(3)
        deadline = time.monotonic() + 3
        matched = False
        while time.monotonic() < deadline:
            publisher.publish(frame(node, 9))
            time.sleep(0.02)
            with node.lock:
                matched = node.latest is not None and node.latest.data[0] == 9
            if matched:
                break
        assert matched, 'Image callback was blocked by inference.'
    finally:
        node.inference_timer.cancel()
        release.set()
        executor.shutdown()
        thread.join(timeout=3)
        source.destroy_node()


@pytest.mark.parametrize('name,value', [
    ('inference_fps', 0.0), ('inference_fps', float('nan')),
    ('confidence_threshold', 1.1), ('model_path', ''), ('image_topic', ''),
])
def test_invalid_parameters_rejected_before_model_load(monkeypatch, name, value):
    loader = Mock()
    monkeypatch.setattr(module, 'load_model', loader)
    context = Context()
    rclpy.init(context=context)
    try:
        with pytest.raises(ValueError):
            module.ChairDetection(context=context, parameter_overrides=[Parameter(name, value=value)])
        loader.assert_not_called()
    finally:
        context.shutdown()


def test_cuda_required_and_wrong_model_rejected(monkeypatch):
    torch = SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: False))
    model = Mock(names={56: 'gate'})
    yolo = Mock(return_value=model)
    monkeypatch.setitem(sys.modules, 'torch', torch)
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=yolo))
    with pytest.raises(RuntimeError, match='CUDA'):
        module.load_model('yolov8n.pt')
    yolo.assert_not_called()
    torch.cuda.is_available = lambda: True
    with pytest.raises(ValueError, match='COCO'):
        module.load_model('gate.pt')
    model.names = {i: ('chair' if i == 56 else 'other') for i in range(80)}
    assert module.load_model('yolov8n.pt') is model
    model.to.assert_called_once_with('cuda:0')


def test_reporting_uses_completed_frames_per_elapsed_second(detector, monkeypatch, capsys):
    node, _ = detector
    logger = Mock()
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    monkeypatch.setattr(module.time, 'monotonic', lambda: node.report_time + 2)
    node.count, node.completed = 3, 10
    node.report()
    assert capsys.readouterr().out == 'Chairs detected: 3 | FPS: 5.0\n'
    assert node.completed == 0
    node.report()
    logger.warning.assert_called_once()
