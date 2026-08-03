"""Generate a deterministic Nav2 occupancy map from the Test Zone mesh."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, cos, floor, hypot, sin
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ElementTree


RESOLUTION_M = 0.05
SLICE_HEIGHT_M = 0.20
MAP_PADDING_M = 0.60
WALL_HALF_WIDTH_M = 0.06
FREE = 254
UNKNOWN = 205
OCCUPIED = 0


@dataclass(frozen=True)
class MapGeometry:
    origin_x: float
    origin_y: float
    width: int
    height: int
    minimum_x: float
    minimum_y: float
    maximum_x: float
    maximum_y: float


@dataclass(frozen=True)
class _Pose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


def generate_test_zone_map(
    model_sdf: str | Path,
    output_directory: str | Path | None = None,
) -> str:
    """Slice the installed collision mesh and return a generated map YAML."""
    source = Path(model_sdf)
    if not source.is_file():
        raise FileNotFoundError(f'Test Zone model not found: {source}')
    mesh_path, pose = _collision_mesh(source)
    vertices, faces = _load_collada(mesh_path)
    transformed = tuple(_transform(vertex, pose) for vertex in vertices)
    geometry = _map_geometry(transformed)

    pixels = bytearray([UNKNOWN]) * (geometry.width * geometry.height)
    _paint_interior(pixels, geometry)
    for start, end in _cross_section(transformed, faces):
        _paint_segment(pixels, geometry, start, end)
    _paint_perimeter(pixels, geometry)

    target = Path(
        output_directory
        or tempfile.mkdtemp(prefix='omokai-test-zone-map-')
    )
    target.mkdir(parents=True, exist_ok=True)
    image_path = target / 'test_zone.pgm'
    image_path.write_bytes(
        (
            f'P5\n{geometry.width} {geometry.height}\n255\n'.encode('ascii')
            + bytes(pixels)
        )
    )
    yaml_path = target / 'test_zone.yaml'
    yaml_path.write_text(
        '\n'.join(
            (
                f'image: {image_path}',
                'mode: trinary',
                f'resolution: {RESOLUTION_M}',
                f'origin: [{geometry.origin_x}, {geometry.origin_y}, 0.0]',
                'negate: 0',
                'occupied_thresh: 0.65',
                'free_thresh: 0.25',
                '',
            )
        ),
        encoding='utf-8',
    )
    return str(yaml_path)


def _collision_mesh(model_sdf: Path) -> tuple[Path, _Pose]:
    root = ElementTree.parse(model_sdf).getroot()
    model = root.find('model')
    collision = None if model is None else model.find('./link/collision')
    if collision is None:
        raise ValueError('Test Zone SDF has no collision mesh')
    uri = collision.findtext('./geometry/mesh/uri', '').strip()
    prefix = 'model://test_zone/'
    if not uri.startswith(prefix):
        raise ValueError(f'unsupported Test Zone mesh URI: {uri!r}')
    mesh_path = model_sdf.parent / uri[len(prefix):]
    if not mesh_path.is_file():
        raise FileNotFoundError(f'Test Zone mesh not found: {mesh_path}')
    values = tuple(float(value) for value in collision.findtext('pose', '').split())
    if len(values) != 6:
        raise ValueError('Test Zone collision pose must have six values')
    return mesh_path, _Pose(*values)


def _load_collada(
    mesh_path: Path,
) -> tuple[tuple[tuple[float, float, float], ...], tuple[tuple[int, ...], ...]]:
    root = ElementTree.parse(mesh_path).getroot()
    namespace = root.tag.partition('}')[0].lstrip('{')
    tag = lambda name: f'{{{namespace}}}{name}' if namespace else name
    mesh = root.find(f'.//{tag("geometry")}/{tag("mesh")}')
    if mesh is None:
        raise ValueError('COLLADA file has no geometry mesh')

    position_source = None
    vertices_element = mesh.find(tag('vertices'))
    if vertices_element is not None:
        position_input = next(
            (
                item
                for item in vertices_element.findall(tag('input'))
                if item.get('semantic') == 'POSITION'
            ),
            None,
        )
        if position_input is not None:
            position_source = position_input.get('source', '').lstrip('#')
    source = next(
        (
            item
            for item in mesh.findall(tag('source'))
            if item.get('id') == position_source
        ),
        None,
    )
    array = None if source is None else source.find(tag('float_array'))
    if array is None or not array.text:
        raise ValueError('COLLADA mesh has no position array')
    values = tuple(float(value) for value in array.text.split())
    if len(values) % 3:
        raise ValueError('COLLADA position array is not xyz triples')
    vertices = tuple(
        (values[index], values[index + 1], values[index + 2])
        for index in range(0, len(values), 3)
    )

    faces: list[tuple[int, ...]] = []
    for primitive_name in ('polylist', 'triangles'):
        for primitive in mesh.findall(tag(primitive_name)):
            inputs = primitive.findall(tag('input'))
            vertex_input = next(
                (item for item in inputs if item.get('semantic') == 'VERTEX'),
                None,
            )
            if vertex_input is None:
                continue
            offset = int(vertex_input.get('offset', '0'))
            stride = max(int(item.get('offset', '0')) for item in inputs) + 1
            indices_text = primitive.findtext(tag('p'), '')
            raw_indices = tuple(int(value) for value in indices_text.split())
            if primitive_name == 'triangles':
                counts = (3,) * int(primitive.get('count', '0'))
            else:
                counts = tuple(
                    int(value)
                    for value in primitive.findtext(tag('vcount'), '').split()
                )
            cursor = 0
            for count in counts:
                face = tuple(
                    raw_indices[cursor + vertex * stride + offset]
                    for vertex in range(count)
                )
                faces.append(face)
                cursor += count * stride
    if not faces:
        raise ValueError('COLLADA mesh has no polygon faces')
    return vertices, tuple(faces)


def _transform(
    point: tuple[float, float, float],
    pose: _Pose,
) -> tuple[float, float, float]:
    x, y, z = point
    cr, sr = cos(pose.roll), sin(pose.roll)
    cp, sp = cos(pose.pitch), sin(pose.pitch)
    cy, sy = cos(pose.yaw), sin(pose.yaw)
    return (
        pose.x + (cy * cp) * x + (cy * sp * sr - sy * cr) * y
        + (cy * sp * cr + sy * sr) * z,
        pose.y + (sy * cp) * x + (sy * sp * sr + cy * cr) * y
        + (sy * sp * cr - cy * sr) * z,
        pose.z - sp * x + cp * sr * y + cp * cr * z,
    )


def _map_geometry(
    vertices: tuple[tuple[float, float, float], ...],
) -> MapGeometry:
    minimum_x = min(point[0] for point in vertices)
    minimum_y = min(point[1] for point in vertices)
    maximum_x = max(point[0] for point in vertices)
    maximum_y = max(point[1] for point in vertices)
    origin_x = floor((minimum_x - MAP_PADDING_M) / RESOLUTION_M) * RESOLUTION_M
    origin_y = floor((minimum_y - MAP_PADDING_M) / RESOLUTION_M) * RESOLUTION_M
    width = ceil((maximum_x + MAP_PADDING_M - origin_x) / RESOLUTION_M)
    height = ceil((maximum_y + MAP_PADDING_M - origin_y) / RESOLUTION_M)
    return MapGeometry(
        origin_x,
        origin_y,
        width,
        height,
        minimum_x,
        minimum_y,
        maximum_x,
        maximum_y,
    )


def _cross_section(
    vertices: tuple[tuple[float, float, float], ...],
    faces: tuple[tuple[int, ...], ...],
) -> tuple[
    tuple[tuple[float, float], tuple[float, float]],
    ...,
]:
    segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
    tolerance = 1e-7
    for face in faces:
        points: list[tuple[float, float]] = []
        for index, vertex_index in enumerate(face):
            start = vertices[vertex_index]
            end = vertices[face[(index + 1) % len(face)]]
            start_delta = start[2] - SLICE_HEIGHT_M
            end_delta = end[2] - SLICE_HEIGHT_M
            if abs(start_delta) <= tolerance and abs(end_delta) <= tolerance:
                segments.append(((start[0], start[1]), (end[0], end[1])))
                continue
            if start_delta * end_delta > 0.0:
                continue
            if abs(end[2] - start[2]) <= tolerance:
                continue
            ratio = (SLICE_HEIGHT_M - start[2]) / (end[2] - start[2])
            if -tolerance <= ratio <= 1.0 + tolerance:
                point = (
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                )
                if not any(
                    hypot(point[0] - other[0], point[1] - other[1])
                    <= tolerance
                    for other in points
                ):
                    points.append(point)
        if len(points) == 2:
            segments.append((points[0], points[1]))
        elif len(points) > 2:
            for index in range(len(points)):
                segments.append((points[index - 1], points[index]))
    return tuple(segments)


def _paint_interior(pixels: bytearray, geometry: MapGeometry) -> None:
    minimum_x = _cell_x(geometry, geometry.minimum_x)
    maximum_x = _cell_x(geometry, geometry.maximum_x)
    minimum_y = _cell_y(geometry, geometry.minimum_y)
    maximum_y = _cell_y(geometry, geometry.maximum_y)
    for cell_y in range(minimum_y, maximum_y + 1):
        for cell_x in range(minimum_x, maximum_x + 1):
            _set_pixel(pixels, geometry, cell_x, cell_y, FREE)


def _paint_segment(
    pixels: bytearray,
    geometry: MapGeometry,
    start: tuple[float, float],
    end: tuple[float, float],
) -> None:
    margin = WALL_HALF_WIDTH_M + RESOLUTION_M
    minimum_x = _cell_x(geometry, min(start[0], end[0]) - margin)
    maximum_x = _cell_x(geometry, max(start[0], end[0]) + margin)
    minimum_y = _cell_y(geometry, min(start[1], end[1]) - margin)
    maximum_y = _cell_y(geometry, max(start[1], end[1]) + margin)
    for cell_y in range(max(0, minimum_y), min(geometry.height, maximum_y + 1)):
        world_y = geometry.origin_y + (cell_y + 0.5) * RESOLUTION_M
        for cell_x in range(max(0, minimum_x), min(geometry.width, maximum_x + 1)):
            world_x = geometry.origin_x + (cell_x + 0.5) * RESOLUTION_M
            if _distance_to_segment((world_x, world_y), start, end) <= margin:
                _set_pixel(pixels, geometry, cell_x, cell_y, OCCUPIED)


def _paint_perimeter(pixels: bytearray, geometry: MapGeometry) -> None:
    corners = (
        (geometry.minimum_x, geometry.minimum_y),
        (geometry.maximum_x, geometry.minimum_y),
        (geometry.maximum_x, geometry.maximum_y),
        (geometry.minimum_x, geometry.maximum_y),
    )
    for index in range(len(corners)):
        _paint_segment(pixels, geometry, corners[index - 1], corners[index])


def _distance_to_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    squared_length = delta_x * delta_x + delta_y * delta_y
    if squared_length == 0.0:
        return hypot(point[0] - start[0], point[1] - start[1])
    ratio = max(
        0.0,
        min(
            1.0,
            ((point[0] - start[0]) * delta_x + (point[1] - start[1]) * delta_y)
            / squared_length,
        ),
    )
    projection = (start[0] + ratio * delta_x, start[1] + ratio * delta_y)
    return hypot(point[0] - projection[0], point[1] - projection[1])


def _cell_x(geometry: MapGeometry, world_x: float) -> int:
    return floor((world_x - geometry.origin_x) / RESOLUTION_M)


def _cell_y(geometry: MapGeometry, world_y: float) -> int:
    return floor((world_y - geometry.origin_y) / RESOLUTION_M)


def _set_pixel(
    pixels: bytearray,
    geometry: MapGeometry,
    cell_x: int,
    cell_y: int,
    value: int,
) -> None:
    if 0 <= cell_x < geometry.width and 0 <= cell_y < geometry.height:
        image_row = geometry.height - 1 - cell_y
        pixels[image_row * geometry.width + cell_x] = value
