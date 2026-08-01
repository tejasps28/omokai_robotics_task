import numpy as np
import pytest

from omokai_perception.yolox import decode_person_detections, letterbox
from omokai_perception.yolox import prepare_yolox_input


def test_letterbox_preserves_aspect_ratio_and_uses_expected_padding():
    source = np.zeros((480, 640, 3), dtype=np.uint8)
    output, scale = letterbox(source)

    assert output.shape == (640, 640, 3)
    assert scale == 1.0
    assert np.all(output[480:] == 114.0)


def test_preprocess_swaps_bgr_to_rgb_without_scaling_values():
    source = np.zeros((2, 3, 3), dtype=np.uint8)
    source[0, 0] = (10, 20, 30)

    blob, scale = prepare_yolox_input(source)

    assert blob.shape == (1, 3, 640, 640)
    assert scale == pytest.approx(640 / 3)
    assert blob[0, :, 0, 0].tolist() == [30.0, 20.0, 10.0]


def test_decoder_rejects_unknown_output_shape():
    with pytest.raises(ValueError, match='output shape'):
        decode_person_detections(
            np.zeros((1, 8400, 84), dtype=np.float32),
            1.0,
            (480, 640),
            0.3,
            0.5,
        )


def test_decoder_returns_only_person_class_scores():
    output = np.zeros((1, 8400, 85), dtype=np.float32)
    output[0, 0, 2:4] = np.log([2.0, 4.0])
    output[0, 0, 4] = 0.9
    output[0, 0, 5] = 0.8
    output[0, 1, 4] = 0.99
    output[0, 1, 6] = 0.99

    detections = decode_person_detections(
        output, 1.0, (480, 640), 0.3, 0.5
    )

    assert len(detections) == 1
    assert detections[0].class_id == 0
    assert detections[0].confidence == pytest.approx(0.72)
