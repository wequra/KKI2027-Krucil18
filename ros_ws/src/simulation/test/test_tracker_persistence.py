"""Real target YOLO/ByteTrack test; requires a recorded continuous buoy clip.

BUOY_TRACK_VIDEO must show at least one continuously visible buoy. No motor/ROS
driver starts here. Missing fixture/dependencies are SKIP, never PASS.
"""
import os
from pathlib import Path
import pytest


def test_tracker_persistence():
    clip = os.environ.get('BUOY_TRACK_VIDEO')
    if not clip:
        pytest.skip('NOT EXECUTED — requires Jetson/Jazzy and BUOY_TRACK_VIDEO fixture')
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    from ultralytics import YOLO
    from ament_index_python.packages import get_package_share_directory
    import cv2
    weights = Path(get_package_share_directory('perception')) / 'weights' / 'best.pt'
    assert weights.is_file()
    model = YOLO(str(weights))
    cap = cv2.VideoCapture(clip)
    assert cap.isOpened(), clip
    observed = []
    try:
        for _ in range(20):
            ok, frame = cap.read()
            assert ok, 'Need at least 20 continuous frames with a visible buoy'
            results = model.track(frame, persist=True, tracker='bytetrack.yaml',
                                  conf=.25, iou=.45, verbose=False)
            ids = set()
            for result in results:
                if result.boxes is None or result.boxes.id is None:
                    continue
                for box in result.boxes:
                    label = str(model.names[int(box.cls.item())]).casefold()
                    if label in ('red buoy', 'green buoy'):
                        ids.add(int(box.id.item()))
            observed.append(ids)
        assert set.intersection(*observed), 'No buoy ID persisted through all fixture frames'
        # One synthetic missed frame then the same real frame: no second inference per frame.
        model.track(frame * 0, persist=True, tracker='bytetrack.yaml', conf=.25, verbose=False)
        recovered = model.track(frame, persist=True, tracker='bytetrack.yaml', conf=.25, verbose=False)
        ids = {int(identity) for r in recovered if r.boxes is not None and r.boxes.id is not None
               for identity in r.boxes.id.tolist()}
        assert observed[-1] & ids, 'ID not recovered after one missed observation'
    finally:
        cap.release()
