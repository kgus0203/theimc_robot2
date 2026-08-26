import json
import shutil
from pathlib import Path
from typing import List, Tuple

from plant_inspection_pkg.models import FilePayload, Inspection


class PendingQueue:
    def __init__(self, root_dir: str):
        self.root = Path(root_dir)
        self.root.mkdir(parents=True, exist_ok=True)

    def save(self, inspection: Inspection, files: List[FilePayload]) -> Path:
        target = self.root / inspection.inspection_id
        target.mkdir(parents=True, exist_ok=True)
        (target / "metadata.json").write_text(
            json.dumps(inspection.to_dict(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        for payload in files:
            (target / payload.filename).write_bytes(payload.data)
        return target

    def list_inspections(self) -> List[str]:
        return sorted(path.name for path in self.root.iterdir() if path.is_dir())

    def load(self, inspection_id: str) -> Tuple[Inspection, List[FilePayload]]:
        target = self.root / inspection_id
        metadata = json.loads((target / "metadata.json").read_text(encoding="utf-8"))
        inspection = Inspection.from_dict(metadata)
        files = []
        for capture in inspection.captures:
            for kind, filename in capture.files.items():
                local_filename = Path(filename).name
                path = target / local_filename
                files.append(
                    FilePayload(
                        field_name=f"{capture.viewpoint_id}_{kind}",
                        filename=local_filename,
                        content_type="image/jpeg" if kind == "rgb" else "image/png",
                        data=path.read_bytes(),
                    )
                )
        return inspection, files

    def delete(self, inspection_id: str) -> None:
        shutil.rmtree(self.root / inspection_id)
