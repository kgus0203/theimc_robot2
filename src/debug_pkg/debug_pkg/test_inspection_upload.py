"""Send one synthetic inspection without ROS actions, hardware, or a retry queue."""
import argparse
import json
import math
import sys
import uuid

import cv2
import numpy as np

from plant_inspection_pkg.models import (
    ArmPose, CameraPose, Capture, Environment, FilePayload, Inspection, Location,
    Orientation, Position, utc_offset_timestamp,
)
from plant_inspection_pkg.uploader import UploadError, Uploader


DEFAULT_URL = 'http://192.168.0.72:8080/api/v1/inspections'


def make_test_inspection(location):
    timestamp = utc_offset_timestamp()
    inspection_id = 'TEST_' + uuid.uuid4().hex
    viewpoint = 'debug_test'
    prefix = f'{inspection_id}_{viewpoint}'
    rgb = np.full((240, 320, 3), (60, 120, 30), dtype=np.uint8)
    cv2.putText(rgb, 'UPLOAD TEST', (25, 125), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (255, 255, 255), 2)
    depth = np.full((240, 320), 1000, dtype=np.uint16)
    intrinsics = dict(width=320, height=240, fx=300., fy=300., ppx=160., ppy=120.,
                      model='none', coeffs=[0.] * 5)
    extrinsics = dict(rotation=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                      translation_meters=[0., 0., 0.])
    camera_info = dict(
        schema_version=1, device_serial='TEST_SYNTHETIC', frames_aligned=False,
        color=dict(encoding='bgr8', intrinsics=intrinsics),
        depth=dict(encoding='16UC1', unit='raw_uint16', scale_meters_per_unit=0.001,
                   intrinsics=intrinsics),
        extrinsics=dict(depth_to_color=extrinsics, color_to_depth=extrinsics),
    )
    files = []
    for kind, ext, mime, data in (
        ('rgb', '.jpg', 'image/jpeg', rgb),
        ('depth', '.png', 'image/png', depth),
    ):
        ok, encoded = cv2.imencode(ext, data)
        if not ok:
            raise RuntimeError(f'Failed to encode {kind}')
        files.append(FilePayload(f'{viewpoint}_{kind}', f'{prefix}_{kind}{ext}',
                                 mime, encoded.tobytes()))
    files.append(FilePayload(f'{viewpoint}_camera_info', f'{prefix}_camera_info.json',
                             'application/json', json.dumps(camera_info).encode()))
    capture = Capture(
        viewpoint, timestamp,
        CameraPose('test_frame', Position(0., 0., 1.), Orientation(0., 0., 0., 1.)),
        ArmPose([0.] * 6, [0., 0., 1000., 0., 0., 0.]),
        {key: f'/data/{inspection_id}/{item.filename}'
         for key, item in zip(('rgb', 'depth', 'camera_info'), files)},
        aligned_depth=False,
    )
    inspection = Inspection(inspection_id, timestamp, location,
                            Environment(25.5, 65., 600, 350.), [capture])
    return inspection, files


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upload-url', default=DEFAULT_URL)
    parser.add_argument('--timeout', type=float, default=10.)
    parser.add_argument('--region', default='sj')
    parser.add_argument('--device-number', type=int, default=1)
    parser.add_argument('--bed-id', type=int, default=1)
    parser.add_argument('--hole-id', type=int, default=1)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    try:
        inspection, files = make_test_inspection(Location(
            args.region, args.device_number, args.bed_id, args.hole_id))
        print(f'POST {args.upload_url}', flush=True)
        print(f'Test inspection: {inspection.inspection_id} (synthetic data)', flush=True)
        response = Uploader(args.upload_url, timeout_sec=args.timeout).upload(inspection, files)
    except (UploadError, RuntimeError, ValueError, cv2.error) as exc:
        print(f'UPLOAD FAILED: {exc}', file=sys.stderr)
        return 1
    print('UPLOAD ACCEPTED: server returned HTTP success')
    print(json.dumps(response, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
