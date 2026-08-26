import json
import mimetypes
import uuid
from typing import Dict, List
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from plant_inspection_pkg.models import FilePayload, Inspection


class UploadError(RuntimeError):
    pass


class Uploader:
    def __init__(self, upload_url: str, timeout_sec: float = 15.0):
        self.upload_url = upload_url
        self.timeout_sec = timeout_sec

    def upload(self, inspection: Inspection, files: List[FilePayload]) -> Dict:
        body, content_type = self._build_multipart(inspection, files)
        request = Request(
            self.upload_url,
            data=body,
            headers={"Content-Type": content_type},
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout_sec) as response:
                data = response.read().decode("utf-8")
        except HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise UploadError(f"HTTP {exc.code}: {detail}") from exc
        except URLError as exc:
            raise UploadError(str(exc.reason)) from exc
        except OSError as exc:
            raise UploadError(str(exc)) from exc

        if not data:
            return {"status": "accepted"}
        try:
            return json.loads(data)
        except json.JSONDecodeError:
            return {"status": "accepted", "raw_response": data}

    @staticmethod
    def _build_multipart(inspection: Inspection, files: List[FilePayload]):
        boundary = f"----plant-inspection-{uuid.uuid4().hex}"
        chunks = []

        def add(value: bytes) -> None:
            chunks.append(value)

        metadata = json.dumps(inspection.to_dict(), ensure_ascii=False).encode("utf-8")
        add(f"--{boundary}\r\n".encode())
        add(b'Content-Disposition: form-data; name="metadata"; filename="metadata.json"\r\n')
        add(b"Content-Type: application/json; charset=utf-8\r\n\r\n")
        add(metadata)
        add(b"\r\n")

        for payload in files:
            content_type = payload.content_type or mimetypes.guess_type(payload.filename)[0]
            content_type = content_type or "application/octet-stream"
            add(f"--{boundary}\r\n".encode())
            add(
                (
                    f'Content-Disposition: form-data; name="{payload.field_name}"; '
                    f'filename="{payload.filename}"\r\n'
                ).encode()
            )
            add(f"Content-Type: {content_type}\r\n\r\n".encode())
            add(payload.data)
            add(b"\r\n")

        add(f"--{boundary}--\r\n".encode())
        return b"".join(chunks), f"multipart/form-data; boundary={boundary}"
