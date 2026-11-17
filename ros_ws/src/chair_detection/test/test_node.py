"""Real ROS/CvBridge tests with a fake detector; no weights, camera or GPU required."""

import json
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
from std_msgs.msg import String

from chair_detection import node as module


@pytest.fixture
def detector(monkeypatch):
    context = Context()
    rclpy.init(context=context)
    model = Mock()
    model.names = {56: 'chair', 0: 'person'}
    model.predict.return_value = [SimpleNamespace(boxes=[chair_box(), chair_box()])]
    monkeypatch.setattr(module, 'load_model', lambda path: model)
    node = module.DetectionNode(context=context)
    node.inference_timer.cancel()
    node.report_timer.cancel()
    yield node, model
    node.destroy_node()
    context.shutdown()


def frame(node, value=0):
    return node.bridge.cv2_to_imgmsg(np.full((8, 8, 3), value, dtype=np.uint8), 'bgr8')


def chair_box():
    return object_box(56)


def object_box(class_id, confidence=0.9, xyxy=(1., 2., 6., 7.)):
    return SimpleNamespace(
        cls=SimpleNamespace(item=lambda: class_id),
        conf=SimpleNamespace(item=lambda: confidence),
        xyxy=np.array([xyxy]),
    )


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
    subscriber = node.create_subscription(String, '/vision/detections',
        lambda msg: received.append(json.loads(msg.data)['classes']['chair']['count']), 10)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        for expected in (2, 0):
            boxes = [chair_box() for _ in range(expected)]
            model.predict.return_value = [SimpleNamespace(boxes=boxes)]
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
        payload = json.loads(publisher.publish.call_args.args[0].data)
        assert payload['classes']['chair']['count'] == 2
    finally:
        node.publisher = original


def test_annotated_image_preserves_header_and_matches_inference(detector):
    node, model = detector
    annotated = np.full((8, 8, 3), 77, dtype=np.uint8)
    result = SimpleNamespace(boxes=[chair_box()], plot=Mock(return_value=annotated))
    model.predict.return_value = [result]
    image_publisher = Mock()
    image_publisher.get_subscription_count.return_value = 1
    original = node.image_publisher
    node.image_publisher = image_publisher
    try:
        message = frame(node)
        message.header.frame_id = 'camera_frame'
        message.header.stamp.sec = 123
        node.receive(message)
        node.infer()
        output = image_publisher.publish.call_args.args[0]
        assert output.header == message.header
        assert output.encoding == 'bgr8'
        assert np.array_equal(node.bridge.imgmsg_to_cv2(output, 'bgr8'), annotated)
        result.plot.assert_called_once()
    finally:
        node.image_publisher = original


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


def test_no_viewer_skips_rendering(detector):
    node, _ = detector
    original = node.image_publisher
    node.image_publisher = Mock()
    node.image_publisher.get_subscription_count.return_value = 0
    result = Mock()
    try:
        node.publish_annotated_image(result, frame(node).header)
        result.plot.assert_not_called()
        node.image_publisher.publish.assert_not_called()
    finally:
        node.image_publisher = original


def test_annotation_failure_does_not_interrupt_count(detector):
    node, model = detector
    result = SimpleNamespace(boxes=[chair_box()], plot=Mock(side_effect=RuntimeError('plot')))
    model.predict.return_value = [result]
    original = node.image_publisher
    node.image_publisher = Mock()
    node.image_publisher.get_subscription_count.return_value = 1
    try:
        node.receive(frame(node))
        node.infer()
        assert node.counts == {'chair': 1}
        assert node.completed == 1
        node.image_publisher.publish.assert_not_called()
    finally:
        node.image_publisher = original


def test_multiple_classes_have_counts_boxes_and_confidence(detector):
    node, model = detector
    person = object_box(0, confidence=0.75)
    node.class_ids = [0, 56]
    model.predict.return_value = [SimpleNamespace(boxes=[person, chair_box()])]
    node.receive(frame(node))
    node.infer()
    assert model.predict.call_args.kwargs['classes'] == [0, 56]
    assert node.counts == {'person': 1, 'chair': 1}
    _, classes = node.detect(frame(node))
    assert classes['person'] == {
        'class_id': 0, 'count': 1,
        'instances': [{'bbox_xyxy': [1., 2., 6., 7.], 'confidence': 0.75}],
    }
    model.predict.return_value = [SimpleNamespace(boxes=[chair_box(), chair_box()])]
    _, classes = node.detect(frame(node))
    assert classes['person']['count'] == 0
    assert classes['person']['instances'] == []
    assert classes['chair']['count'] == 2
    assert len(classes['chair']['instances']) == 2


def test_detection_header_and_unrequested_class_filter(detector):
    node, model = detector
    model.predict.return_value = [SimpleNamespace(boxes=[object_box(0), chair_box()])]
    original = node.publisher
    node.publisher = Mock()
    try:
        message = frame(node)
        message.header.frame_id = 'camera_frame'
        message.header.stamp.sec = 123
        message.header.stamp.nanosec = 456
        node.receive(message)
        node.infer()
        payload = json.loads(node.publisher.publish.call_args.args[0].data)
        assert payload['header'] == {
            'frame_id': 'camera_frame', 'stamp': {'sec': 123, 'nanosec': 456}}
        assert list(payload['classes']) == ['chair']
        assert payload['classes']['chair']['count'] == 1
    finally:
        node.publisher = original


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
            module.DetectionNode(context=context, parameter_overrides=[Parameter(name, value=value)])
        loader.assert_not_called()
    finally:
        context.shutdown()


def test_cuda_required_and_class_resolution(monkeypatch, tmp_path):
    checkpoint = tmp_path / 'test.pt'
    checkpoint.write_bytes(b'fake checkpoint')
    torch = SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: False))
    model = Mock(names={56: 'gate'}, task='detect')
    yolo = Mock(return_value=model)
    monkeypatch.setitem(sys.modules, 'torch', torch)
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=yolo))
    with pytest.raises(RuntimeError, match='CUDA'):
        module.load_model(str(checkpoint))
    yolo.assert_not_called()
    torch.cuda.is_available = lambda: True
    with pytest.raises(ValueError, match='not found'):
        module.resolve_classes({0: 'gate'}, ['chair'])
    assert module.resolve_classes({0: 'person', 56: 'chair'}, ['chair', 'person']) == [0, 56]
    assert module.resolve_classes(['person', 'chair'], ['CHAIR']) == [1]
    assert module.resolve_classes(['chair'], [' chair ', 'CHAIR']) == [0]
    with pytest.raises(ValueError, match='nonempty'):
        module.resolve_classes(['chair'], [])
    with pytest.raises(ValueError, match='nonempty'):
        module.resolve_classes(['chair'], [' '])
    model.names = {i: ('chair' if i == 56 else 'other') for i in range(80)}
    assert module.load_model(str(checkpoint)) is model
    model.to.assert_called_once_with('cuda:0')


@pytest.mark.parametrize('kind', ['missing', 'directory', 'empty', 'wrong_suffix'])
def test_bad_checkpoint_rejected_before_yolo(monkeypatch, tmp_path, kind):
    checkpoint = tmp_path / 'weights.pt'
    if kind == 'directory':
        checkpoint.mkdir()
    elif kind == 'empty':
        checkpoint.touch()
    elif kind == 'wrong_suffix':
        checkpoint = tmp_path / 'weights.txt'
        checkpoint.write_text('invalid')
    yolo = Mock()
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=yolo))
    with pytest.raises(ValueError, match='model_path'):
        module.load_model(str(checkpoint))
    yolo.assert_not_called()


def test_incompatible_checkpoint_and_cuda_failure(monkeypatch, tmp_path):
    checkpoint = tmp_path / 'weights.pt'
    checkpoint.write_bytes(b'fake checkpoint')
    model = Mock(task='classify')
    yolo = Mock(return_value=model)
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(
        cuda=SimpleNamespace(is_available=lambda: True)))
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=yolo))
    with pytest.raises(ValueError, match='Expected a detection model'):
        module.load_model(str(checkpoint))
    model.to.assert_not_called()
    yolo.side_effect = OSError('corrupt weights')
    with pytest.raises(RuntimeError, match='Could not load checkpoint.*corrupt weights'):
        module.load_model(str(checkpoint))
    yolo.side_effect = None
    model.task = 'detect'
    model.to.side_effect = RuntimeError('out of memory')
    with pytest.raises(RuntimeError, match='CUDA device 0.*out of memory'):
        module.load_model(str(checkpoint))


def test_label_mismatch_aborts_startup_before_publishers(monkeypatch):
    monkeypatch.setattr(module, 'load_model', lambda path: Mock(names={0: 'gate'}))
    publisher = Mock()
    monkeypatch.setattr(module.DetectionNode, 'create_publisher', publisher)
    context = Context()
    rclpy.init(context=context)
    try:
        with pytest.raises(ValueError, match="Classes not found.*chair.*Available.*gate"):
            module.DetectionNode(context=context)
        # ROS creates its own parameter-event publisher during Node construction.
        assert all(not call.args[1].startswith('/vision/')
                   for call in publisher.call_args_list)
    finally:
        context.shutdown()


def test_reporting_uses_completed_frames_per_elapsed_second(detector, monkeypatch, capsys):
    node, _ = detector
    logger = Mock()
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    monkeypatch.setattr(module.time, 'monotonic', lambda: node.report_time + 2)
    node.counts, node.completed = {'chair': 3, 'person': 1}, 10
    node.report()
    assert capsys.readouterr().out == 'chair: 3 | person: 1 | FPS: 5.0\n'
    assert node.completed == 0
    node.report()
    logger.warning.assert_called_once()
