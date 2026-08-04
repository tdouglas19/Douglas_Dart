from __future__ import annotations

import csv
import json
import math
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path

from douglas_dart.pipeline import _json_ready, _write_csv, _write_json


@dataclass(frozen=True)
class ExampleRow:
    name: str
    value: float
    status: tuple[str, ...]
    optional_value: float | None = None


class PipelineArtifactTests(unittest.TestCase):
    def test_json_ready_converts_dataclasses_paths_and_nonfinite_values(self) -> None:
        converted = _json_ready(
            {
                "row": ExampleRow("point", math.inf, ("unvalidated",)),
                "path": Path("results/example.csv"),
            }
        )
        self.assertEqual(converted["row"]["value"], None)
        self.assertEqual(converted["row"]["status"], ["unvalidated"])
        self.assertEqual(converted["path"], "results/example.csv")

    def test_writers_create_machine_readable_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rows = [
                ExampleRow("a", 1.0, ("ok",)),
                ExampleRow("b", 2.0, ("review", "provisional"), 3.0),
            ]
            csv_path = _write_csv(root / "nested" / "rows.csv", rows)
            json_path = _write_json(root / "nested" / "rows.json", rows)

            with csv_path.open(newline="", encoding="utf-8") as stream:
                csv_rows = list(csv.DictReader(stream))
            self.assertEqual(csv_rows[0]["name"], "a")
            self.assertEqual(json.loads(csv_rows[1]["status"]), ["review", "provisional"])

            json_rows = json.loads(json_path.read_text(encoding="utf-8"))
            self.assertEqual(json_rows[1]["optional_value"], 3.0)


if __name__ == "__main__":
    unittest.main()
