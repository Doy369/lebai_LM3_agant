import unittest
from types import SimpleNamespace
from unittest.mock import patch

from fastapi.testclient import TestClient
from robot_system.web import WebBackendConfig, create_app
from robot_system.web.service import RobotWebService
from robot_system.vision.scene_objects import locate_objects


class CameraSceneTests(unittest.TestCase):
    def test_readonly_camera_observation_does_not_enable_robot_writes(self):
        result = {'success': True, 'message': 'identified', 'data': {'objects': [{'label': '香蕉'}], 'motion_allowed': False}}
        with patch.object(RobotWebService, 'recognize_camera_objects', return_value=result) as recognize:
            with TestClient(create_app(WebBackendConfig(dry_run=True, read_only=True))) as client:
                response = client.get('/api/camera/objects?target=banana')
                self.assertEqual(response.status_code, 200)
                self.assertEqual(response.headers['cache-control'], 'no-store')
                self.assertFalse(response.json()['data']['motion_allowed'])
                recognize.assert_called_once_with('banana')
                self.assertEqual(client.post('/api/pipeline/grasp-from-camera', json={}).status_code, 403)
                self.assertEqual(client.post('/api/robot/open-gripper').status_code, 403)

    def test_observation_position_is_uncompensated_measured_surface(self):
        snapshot = SimpleNamespace(image_size_wh=[1280, 720], sample_depth_in_bbox=lambda bbox: {'u': 110, 'v': 210, 'depth_m': .9})
        transformer = SimpleNamespace(assert_image_size=lambda size: None,
            pixel_to_base=lambda **kw: SimpleNamespace(x=kw['u'] / 1000, y=kw['v'] / 1000, z=kw['depth']))
        objects, skipped = locate_objects(snapshot, transformer, [{'className': 'banana', 'label': '香蕉', 'confidence': .8, 'bbox_xyxy': [100, 200, 180, 280]}])
        self.assertEqual(objects[0]['positionBaseM'], [.11, .21, .9])
        self.assertEqual(objects[0]['color'], '#efc52d')
        self.assertEqual(objects[0]['pointKind'], 'surface')
        self.assertEqual(objects[0]['sizeM'][2], .04)
        self.assertEqual(skipped, [])

    def test_missing_depth_is_not_replaced_with_an_invented_position(self):
        snapshot = SimpleNamespace(image_size_wh=[1280, 720], sample_depth_in_bbox=lambda bbox: None)
        transformer = SimpleNamespace(assert_image_size=lambda size: None)
        objects, skipped = locate_objects(snapshot, transformer, [{'className': 'banana', 'label': '香蕉', 'bbox_xyxy': [100, 200, 180, 280]}])
        self.assertEqual(objects, [])
        self.assertEqual(skipped[0]['reason'], '没有有效深度')


if __name__ == '__main__':
    unittest.main()
