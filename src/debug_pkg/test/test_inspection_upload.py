"""Check the real multipart HTTP request against a local test server."""
import json
import threading
from email.parser import BytesParser
from email.policy import default
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import pytest

from debug_pkg.test_inspection_upload import main


@pytest.mark.parametrize('status,expected', [(201, 0), (422, 1)])
def test_upload(status, expected, capsys):
    received = []

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            body = self.rfile.read(int(self.headers['Content-Length']))
            received.append((self.path, self.headers['Content-Type'], body))
            self.send_response(status)
            self.end_headers()
            self.wfile.write(b'{"message":"test response"}')

        def log_message(self, *args):
            pass

    server = HTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        assert main(['--upload-url', f'http://127.0.0.1:{server.server_port}/api/v1/inspections']) == expected
    finally:
        server.shutdown()
        thread.join()
        server.server_close()
    assert len(received) == 1
    path, content_type, body = received[0]
    assert path == '/api/v1/inspections'
    message = BytesParser(policy=default).parsebytes(
        f'Content-Type: {content_type}\r\n\r\n'.encode() + body)
    parts = {part.get_param('name', header='content-disposition'): part
             for part in message.iter_parts()}
    metadata = json.loads(parts['metadata'].get_payload(decode=True))
    assert metadata['inspection_id'].startswith('TEST_')
    assert metadata['environment']['solar_radiation_w_m2'] == 350.
    assert 'illuminance_lux' not in metadata['environment']
    for key, dtype in [('rgb', np.uint8), ('depth', np.uint16)]:
        part = parts['debug_test_' + key]
        decoded = cv2.imdecode(np.frombuffer(part.get_payload(decode=True), np.uint8),
                               cv2.IMREAD_UNCHANGED)
        assert decoded.dtype == dtype
        assert decoded.shape[:2] == (240, 320)
        assert metadata['captures'][0]['files'][key].endswith(part.get_filename())
    output = capsys.readouterr()
    assert ('UPLOAD ACCEPTED' in output.out) if expected == 0 else ('HTTP 422' in output.err)
