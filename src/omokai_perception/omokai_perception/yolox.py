"""Small OpenCV-DNN adapter for the pinned OpenCV Zoo YOLOX-S model."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from omokai_perception.model import AttributeDecision
from omokai_perception.model import BoundingBox
from omokai_perception.model import PersonCandidate

@dataclass(frozen=True)
class Detection:
    """One axis-aligned image detection in source-image pixels."""

    x: int
    y: int
    width: int
    height: int
    confidence: float
    class_id: int


def letterbox(image: np.ndarray, size: int = 640) -> tuple[np.ndarray, float]:
    """Resize without distortion and pad on the right and bottom."""
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('expected an HxWx3 image')
    ratio = min(size / image.shape[0], size / image.shape[1])
    resized_width = int(image.shape[1] * ratio)
    resized_height = int(image.shape[0] * ratio)
    padded = np.full((size, size, 3), 114.0, dtype=np.float32)
    padded[:resized_height, :resized_width] = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    ).astype(np.float32)
    return padded, ratio


def _grid(size: int = 640) -> tuple[np.ndarray, np.ndarray]:
    grids = []
    expanded_strides = []
    for stride in (8, 16, 32):
        count = size // stride
        x_values, y_values = np.meshgrid(np.arange(count), np.arange(count))
        grid = np.stack((x_values, y_values), axis=2).reshape(1, -1, 2)
        grids.append(grid)
        expanded_strides.append(
            np.full((1, grid.shape[1], 1), stride, dtype=np.float32)
        )
    return np.concatenate(grids, axis=1), np.concatenate(expanded_strides, axis=1)


def prepare_yolox_input(
    bgr_image: np.ndarray,
) -> tuple[np.ndarray, float]:
    """Prepare the OpenCV Zoo YOLOX input without rescaling pixel values.

    The pinned 2022nov export expects the same input as OpenCV Zoo's
    ``blobFromImage(..., swapRB=True)`` example: RGB channel order and the
    original 0--255 value range.  Normalization is part of the exported
    network.
    """

    padded, scale = letterbox(bgr_image)
    rgb = padded[:, :, ::-1]
    blob = np.ascontiguousarray(
        np.transpose(rgb, (2, 0, 1))[np.newaxis, ...],
        dtype=np.float32,
    )
    return blob, scale


def decode_person_detections(
    output: np.ndarray,
    scale: float,
    source_shape: tuple[int, int],
    confidence_threshold: float,
    nms_threshold: float,
) -> list[Detection]:
    """Decode raw YOLOX output and retain only COCO class zero (person)."""
    predictions = np.asarray(output, dtype=np.float32)
    if predictions.ndim == 2:
        predictions = predictions[np.newaxis, ...]
    if predictions.ndim != 3 or predictions.shape[2] != 85:
        raise ValueError(f'unexpected YOLOX output shape {predictions.shape}')

    grid, strides = _grid()
    if predictions.shape[1] != grid.shape[1]:
        raise ValueError(f'unexpected YOLOX proposal count {predictions.shape[1]}')
    decoded = predictions.copy()
    decoded[:, :, :2] = (decoded[:, :, :2] + grid) * strides
    decoded[:, :, 2:4] = np.exp(decoded[:, :, 2:4]) * strides
    decoded = decoded[0]

    scores = decoded[:, 4] * decoded[:, 5]
    keep = scores >= confidence_threshold
    decoded = decoded[keep]
    scores = scores[keep]
    if decoded.size == 0:
        return []

    boxes = np.empty((decoded.shape[0], 4), dtype=np.float32)
    boxes[:, 0] = decoded[:, 0] - decoded[:, 2] / 2.0
    boxes[:, 1] = decoded[:, 1] - decoded[:, 3] / 2.0
    boxes[:, 2:4] = decoded[:, 2:4]
    indices = cv2.dnn.NMSBoxes(
        boxes.tolist(),
        scores.tolist(),
        confidence_threshold,
        nms_threshold,
    )
    if len(indices) == 0:
        return []

    source_height, source_width = source_shape
    detections = []
    for index in np.asarray(indices).reshape(-1):
        x, y, width, height = boxes[int(index)] / scale
        x0 = max(0, min(source_width - 1, int(round(x))))
        y0 = max(0, min(source_height - 1, int(round(y))))
        x1 = max(x0 + 1, min(source_width, int(round(x + width))))
        y1 = max(y0 + 1, min(source_height, int(round(y + height))))
        detections.append(
            Detection(
                x=x0,
                y=y0,
                width=x1 - x0,
                height=y1 - y0,
                confidence=float(scores[int(index)]),
                class_id=0,
            )
        )
    return sorted(detections, key=lambda item: item.confidence, reverse=True)


class YoloXPersonDetector:
    """CPU detector using the fixed 640x640 OpenCV Zoo YOLOX-S export."""

    def __init__(
        self,
        model_path: str,
        confidence_threshold: float = 0.30,
        nms_threshold: float = 0.50,
    ) -> None:
        self._network = cv2.dnn.readNet(model_path)
        self._network.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self._network.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self._confidence_threshold = confidence_threshold
        self._nms_threshold = nms_threshold

    def detect(self, bgr_image: np.ndarray) -> list[Detection]:
        blob, scale = prepare_yolox_input(bgr_image)
        self._network.setInput(blob)
        output = self._network.forward(
            self._network.getUnconnectedOutLayersNames()
        )[0]
        return decode_person_detections(
            output,
            scale,
            bgr_image.shape[:2],
            self._confidence_threshold,
            self._nms_threshold,
        )


def annotate(image: np.ndarray, detections: list[Detection]) -> np.ndarray:
    """Return a copy with clear person boxes and confidence labels."""
    result = image.copy()
    for detection in detections:
        start = (detection.x, detection.y)
        end = (
            detection.x + detection.width,
            detection.y + detection.height,
        )
        cv2.rectangle(result, start, end, (0, 255, 0), 2)
        label = f'person {detection.confidence:.2f}'
        baseline_y = max(18, detection.y)
        cv2.rectangle(
            result,
            (detection.x, baseline_y - 18),
            (min(result.shape[1] - 1, detection.x + 130), baseline_y),
            (0, 255, 0),
            -1,
        )
        cv2.putText(
            result,
            label,
            (detection.x + 3, baseline_y - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
    return result


def annotate_selection(
    image: np.ndarray,
    candidates: list[PersonCandidate],
    selected_box: BoundingBox | None,
    confirmed: bool,
) -> np.ndarray:
    """Render accepted targets green and rejected/uncertain people red."""
    result = image.copy()
    for candidate in candidates:
        selected = selected_box == candidate.box
        accepted = candidate.decision is AttributeDecision.ACCEPTED
        color = (0, 200, 0) if accepted else (0, 0, 255)
        thickness = 3 if selected else 2
        start = (candidate.box.x, candidate.box.y)
        end = (
            candidate.box.x + candidate.box.width,
            candidate.box.y + candidate.box.height,
        )
        cv2.rectangle(result, start, end, color, thickness)
        if selected and confirmed:
            state = 'TARGET'
        else:
            state = candidate.decision.value.upper()
        label = (
            f'{state} {candidate.observed_color} '
            f'{candidate.attribute_score:.2f}'
        )
        baseline_y = max(18, candidate.box.y)
        label_width = min(result.shape[1] - 1, candidate.box.x + 180)
        cv2.rectangle(
            result,
            (candidate.box.x, baseline_y - 18),
            (label_width, baseline_y),
            color,
            -1,
        )
        cv2.putText(
            result,
            label,
            (candidate.box.x + 3, baseline_y - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return result
