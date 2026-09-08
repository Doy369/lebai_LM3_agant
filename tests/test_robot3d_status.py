"""Read-only API regression: fake controller, no hardware or action endpoints."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from fastapi.testclient import TestClient
from robot_system.web import WebBackendConfig, create_app


class ReadOnlyStatusTests(unittest.TestCase):
    def test_feedback_metadata_auth_and_no_actions(self):
        calls = []
        class Controller:
            command_log = []
            def connect(self): calls.append('connect')
            def get_robot_status_summary(self):
                calls.append('status')
                return {'is_connected': True, 'robot_state': 'dry_run'}
            def get_kin_data(self):
                calls.append('kin_data')
                return {'actual_joint_pose': [0, -.5, .5, 0, 0, 0]}
            def disconnect(self): calls.append('disconnect')
        with tempfile.TemporaryDirectory() as temp:
            config = WebBackendConfig(dry_run=True, web_token='fixture-token', runtime_state_file=Path(temp)/'state.json')
            app = create_app(config)
            with patch.object(app.state.service, '_build_status_controller', return_value=Controller()):
                with TestClient(app) as client:
                    self.assertEqual(client.get('/api/robot/status').status_code, 401)
                    response = client.get('/api/robot/status', headers={'X-Lebai-Token': 'fixture-token'})
                    self.assertEqual(response.status_code, 200)
                    self.assertEqual(response.headers['cache-control'], 'no-store')
                    data = response.json()['data']
                    self.assertEqual(data['joint_unit'], 'radian')
                    self.assertEqual(data['feedback_time_source'], 'server_read_completion')
                    self.assertIsInstance(data['feedback_read_at_unix_ms'], int)
                    self.assertEqual(len(data['kin_data']['actual_joint_pose']), 6)
                    self.assertEqual(client.get('/ui/robot3d/assets/Lebai_LM3.glb').status_code, 200)
        self.assertEqual(calls, ['connect', 'status', 'kin_data', 'disconnect'])

if __name__ == '__main__':
    unittest.main()
