import json
import math
import pandas as pd
import numpy as np
from scipy.spatial import transform
from csaps import csaps


class SimulationPipeline:
    def __init__(self, metadata_path, measurements_path):
        with open(metadata_path) as metadata:
            self._metadata = json.load(metadata)

        input_measurements = pd.read_csv(measurements_path)
        self._measurements = {}
        for marker in self._metadata['markers']:
            self._measurements[marker] = input_measurements[input_measurements['marker_id'] == marker]

    def set_camera_metadata(self, fov, sensor_width, resolution, sensor_variance,
                            calibrated_position=None, calibrated_rotation=None):
        # This is the rotation that needs to be applied to map the Unity coordinate system to a NED system.
        to_ned = transform.Rotation.from_quat((1/math.sqrt(2), 1/math.sqrt(2), 0, 0))

        for i in range(len(self._metadata['cameras'])):
            self._metadata['cameras'][i]['fieldOfView'] = fov
            self._metadata['cameras'][i]['sensorWidth'] = sensor_width
            self._metadata['cameras'][i]['sensorVariance'] = sensor_variance
            self._metadata['cameras'][i]['resolution'] = resolution

            if calibrated_position is not None:
                self._metadata['cameras'][i]['calibratedPosition'] = calibrated_position

            if calibrated_rotation is not None:
                self._metadata['cameras'][i]['calibratedRotation'] = calibrated_rotation

            q0 = self._metadata['cameras'][i]['rotation']['q0']
            q1 = self._metadata['cameras'][i]['rotation']['q1']
            q2 = self._metadata['cameras'][i]['rotation']['q2']
            q3 = self._metadata['cameras'][i]['rotation']['q3']
            # Unity uses a left-handed coordinate system but common in robotics is a right handed system. So to not to
            # modify the GTSAM factors we use we transform the rotations to the new system.
            unity_rot = transform.Rotation.from_quat((q1, q3, q2, q0))
            ned_rot = (unity_rot*to_ned).as_quat()
            self._metadata['cameras'][i]['rotation']['q0'] = ned_rot[3]
            self._metadata['cameras'][i]['rotation']['q1'] = ned_rot[0]
            self._metadata['cameras'][i]['rotation']['q2'] = ned_rot[1]
            self._metadata['cameras'][i]['rotation']['q3'] = ned_rot[2]

    def transform_measurements(self):
        for marker in self._measurements:
            tmp_pos = self._measurements[marker]['z_true'].copy()
            self._measurements[marker]['z_true'] = -self._measurements[marker]['y_true']
            self._measurements[marker]['y_true'] = -tmp_pos

            tmp_rot = self._measurements[marker]['q3'].copy()
            self._measurements[marker]['q1'] = -self._measurements[marker]['q1']
            self._measurements[marker]['q3'] = self._measurements[marker]['q2']
            self._measurements[marker]['q2'] = tmp_rot

    def fit_smoothing_spline(self, smoothing_factor=1-1e-6):
        for marker in self._measurements:
            spline = csaps(
                self._measurements[marker]['t'],
                (
                    self._measurements[marker]['x_true'],
                    self._measurements[marker]['y_true'],
                    self._measurements[marker]['z_true'],
                ),
                self._measurements[marker]['t'],
                smooth=smoothing_factor
            )
            spline = np.transpose(spline)
            self._measurements[marker]['x_true'] = spline[:, 0]
            self._measurements[marker]['y_true'] = spline[:, 1]
            self._measurements[marker]['z_true'] = spline[:, 2]

    def compute_kinematics(self):
        for marker in self._measurements:
            for axis in ('x', 'y', 'z'):
                vel = np.gradient(self._measurements[marker]['{}_true'.format(axis)], self._measurements[marker]['t'])
                accel = np.gradient(vel, self._measurements[marker]['t'])
                self._measurements[marker]['{}_vel'.format(axis)] = vel
                self._measurements[marker]['{}_accel'.format(axis)] = accel
