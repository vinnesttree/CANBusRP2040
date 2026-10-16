# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0
"""schema.json, the installed phoenix6 package and the generated firmware sources agree."""

import re
import subprocess
import sys
import unittest
from pathlib import Path

from phoenix6_bridge.schema import compare_with_installed, load_schema, schema_hash

ROOT = Path(__file__).resolve().parents[3]
HEADER = ROOT / "include" / "canbus_rp2040" / "phoenix6_generated.h"


class SchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.schema = load_schema()

    def test_matches_installed_phoenix6(self):
        self.assertEqual(compare_with_installed(self.schema), [])

    def test_firmware_hash(self):
        m = re.search(r"#define PHOENIX6_SCHEMA_HASH 0x([0-9a-f]+)U", HEADER.read_text())
        self.assertIsNotNone(m)
        self.assertEqual(int(m.group(1), 16), schema_hash(self.schema))

    def test_generated_sources_up_to_date(self):
        subprocess.run(
            [sys.executable, str(ROOT / "scripts" / "gen_phoenix6.py"), "--check"], check=True
        )

    def test_ids_are_dense(self):
        for key in ("controls", "configs", "signals"):
            ids = [item["id"] for item in self.schema[key]]
            self.assertEqual(ids, list(range(1, len(ids) + 1)), key)

    def test_no_documentation_text(self):
        # Only interoperability facts are stored, no CTR Electronics doc text
        def walk(obj):
            if isinstance(obj, dict):
                for k, v in obj.items():
                    self.assertNotIn(k, ("doc", "summary", "description"))
                    walk(v)
            elif isinstance(obj, list):
                for v in obj:
                    walk(v)

        walk(self.schema)


if __name__ == "__main__":
    unittest.main()
