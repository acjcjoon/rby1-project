"""JSON-file implementation used as a mock external server."""
from __future__ import annotations
import json
import os
from pathlib import Path
from typing import Any
from .base import ExternalDataSource

class JsonDataSource(ExternalDataSource):
    def __init__(self, input_path: str | Path, output_path: str | Path, events_path: str | Path) -> None:
        self.input_path = Path(input_path)
        self.output_path = Path(output_path)
        self.events_path = Path(events_path)

    def read_snapshot(self) -> dict[str, Any]:
        with self.input_path.open(encoding='utf-8') as stream:
            data = json.load(stream)
        if data.get('schema_version') != 1:
            raise ValueError('unsupported mock-server schema_version')
        barcodes = [plate['barcode'] for batch in data.get('batches', []) for plate in batch.get('plates', {}).values()]
        if len(barcodes) != len(set(barcodes)):
            raise ValueError('plate barcodes must be unique')
        return data

    def write_scheduler_state(self, state: dict[str, Any]) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_path.with_suffix(self.output_path.suffix + '.tmp')
        with temporary.open('w', encoding='utf-8', newline='\n') as stream:
            json.dump(state, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        os.replace(temporary, self.output_path)

    def append_events(self, events: list[dict[str, Any]]) -> None:
        if not events:
            return
        self.events_path.parent.mkdir(parents=True, exist_ok=True)
        with self.events_path.open('a', encoding='utf-8', newline='\n') as stream:
            for event in events:
                stream.write(json.dumps(event, ensure_ascii=False, separators=(',', ':')) + '\n')
