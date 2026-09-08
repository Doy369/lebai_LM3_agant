"""Loopback-only fixture server: no SDK imports; all POSTs rejected."""
import json
import math
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]

class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT / 'robot_system/web/static'), **kwargs)

    def do_GET(self):
        path = urlsplit(self.path).path
        if '/api/' in path:
            if path.startswith('/offline/'):
                self.send_error(503, 'Simulated disconnect'); return
            data = {}
            if path.endswith('/health'):
                data = {'dry_run': True, 'status': 'fixture_no_hardware'}
            if path.endswith('/robot/status'):
                data = {'dry_run': True, 'is_connected': True, 'joint_unit': 'radian',
                        'feedback_time_source': 'server_read_completion',
                        'feedback_read_at_unix_ms': int(time.time()*1000) - (10000 if path.startswith('/stale/') else 0),
                        'kin_data': {'actual_joint_pose': [math.sin(time.time())*.4, -.5, .5, 0, .2, 0]}}
            body = json.dumps({'success': True, 'data': data}).encode()
            self.send_response(200); self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store'); self.end_headers(); self.wfile.write(body)
            return
        if path.startswith('/ui/'):
            self.path = self.path[3:]
        super().do_GET()

    def do_POST(self):
        self.send_error(405, 'Fixture server never sends robot actions')

if __name__ == '__main__':
    print('Fixture console: http://127.0.0.1:8765/ui/', flush=True)
    ThreadingHTTPServer(('127.0.0.1', 8765), Handler).serve_forever()
