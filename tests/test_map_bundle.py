import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from map_bundle import (  # noqa: E402
    COMMON_ASSETS,
    PROFILE_ASSETS,
    REQUIRED_ASSETS,
    convert_v2_bundle,
    create_bundle,
    manifest_key,
    sha256_file,
    validate_bundle,
)


class MapBundleTest(unittest.TestCase):
    def make_source(self, root: Path, profile: str = "visual") -> tuple[Path, Path]:
        source = root / f"{profile}-source"
        source.mkdir()
        filenames = tuple(dict.fromkeys(COMMON_ASSETS[:-1] + PROFILE_ASSETS[profile]))
        for filename in filenames:
            content = "{}\n" if filename.endswith(".json") else f"fixture:{filename}\n"
            (source / filename).write_text(content, encoding="utf-8")
        metadata = root / f"{profile}-metadata-input.json"
        metadata.write_text(json.dumps({
            "motion_output_connected": False,
            "source_bag_sha256": "a" * 64,
            "git_commit": "b" * 40,
        }), encoding="utf-8")
        return source, metadata

    def make_v2_bundle(self, root: Path, map_id: str = "legacy-01") -> Path:
        source, metadata_input = self.make_source(root)
        bundle = root / map_id
        bundle.mkdir()
        for filename in REQUIRED_ASSETS:
            if filename != "metadata.json":
                shutil.copy2(source / filename, bundle / filename)
        metadata = json.loads(metadata_input.read_text(encoding="utf-8"))
        metadata.update({
            "map_id": map_id,
            "created_at": "2026-08-20T01:02:03+00:00",
            "schema_version": 2,
        })
        (bundle / "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        assets = {}
        manifest = {
            "schema_version": 2,
            "map_id": map_id,
            "created_at": metadata["created_at"],
        }
        for filename in REQUIRED_ASSETS:
            path = bundle / filename
            digest = sha256_file(path)
            assets[filename] = {"sha256": digest, "size_bytes": path.stat().st_size}
            manifest[manifest_key(filename)] = digest
        manifest["assets"] = assets
        (bundle / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return bundle

    @staticmethod
    def tree_hashes(root: Path) -> dict[str, str]:
        return {
            path.relative_to(root).as_posix(): sha256_file(path)
            for path in sorted(root.rglob("*"))
            if path.is_file()
        }

    def test_create_and_validate_visual_v3(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, metadata = self.make_source(root)
            bundle = root / "map-01"
            create_bundle(source, bundle, "map-01", metadata)
            result = validate_bundle(bundle, "map-01", "visual")
            self.assertEqual(result["schema_version"], 3)
            self.assertEqual(result["profile"], "visual")
            self.assertEqual(set(result["assets"]), {"common", "visual", "lidar"})
            self.assertEqual(set(result["assets"]["common"]), set(COMMON_ASSETS))
            self.assertEqual(set(result["assets"]["visual"]), set(PROFILE_ASSETS["visual"]))
            self.assertEqual(result["assets"]["lidar"], {})

    def test_create_and_validate_lidar_v3_without_rtab_asset(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, metadata = self.make_source(root, "lidar")
            bundle = root / "lidar-map"
            create_bundle(source, bundle, "lidar-map", metadata, profile="lidar")
            result = validate_bundle(bundle, "lidar-map", "lidar")
            self.assertEqual(result["profile"], "lidar")
            self.assertEqual(result["assets"]["visual"], {})
            self.assertEqual(set(result["assets"]["lidar"]), set(PROFILE_ASSETS["lidar"]))
            self.assertFalse((bundle / "rtabmap.db").exists())

    def test_rejects_tampered_asset(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, metadata = self.make_source(root)
            bundle = root / "map-01"
            create_bundle(source, bundle, "map-01", metadata)
            with (bundle / "keepout.pgm").open("a", encoding="utf-8") as stream:
                stream.write("tamper")
            with self.assertRaisesRegex(ValueError, "asset_integrity_mismatch:keepout.pgm"):
                validate_bundle(bundle, "map-01", "visual")

    def test_rejects_wrong_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, metadata = self.make_source(root)
            bundle = root / "map-01"
            create_bundle(source, bundle, "map-01", metadata)
            with self.assertRaisesRegex(ValueError, "bundle_profile_mismatch"):
                validate_bundle(bundle, "map-01", "lidar")

    def test_rejects_missing_common_asset(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, metadata = self.make_source(root)
            bundle = root / "map-01"
            create_bundle(source, bundle, "map-01", metadata)
            (bundle / "map.yaml").unlink()
            with self.assertRaisesRegex(ValueError, "missing_or_empty:map.yaml"):
                validate_bundle(bundle, "map-01", "visual")

    def test_rejects_missing_backend_asset(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, metadata = self.make_source(root, "lidar")
            bundle = root / "lidar-map"
            create_bundle(source, bundle, "lidar-map", metadata, profile="lidar")
            (bundle / "backend.yaml").unlink()
            with self.assertRaisesRegex(ValueError, "missing_or_empty:backend.yaml"):
                validate_bundle(bundle, "lidar-map", "lidar")

    def test_requires_disconnected_motion_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, metadata = self.make_source(root)
            metadata.write_text(json.dumps({"motion_output_connected": True}), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "motion_output_disconnected"):
                create_bundle(source, root / "map-01", "map-01", metadata)

    def test_rejects_nonstandard_json_constants(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, metadata = self.make_source(root)
            metadata.write_text(
                '{"motion_output_connected": false, "quality": NaN}',
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "invalid_json_constant:NaN"):
                create_bundle(source, root / "map-01", "map-01", metadata)

    def test_convert_v2_to_new_v3_preserves_source_and_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = self.make_v2_bundle(root)
            source_before = self.tree_hashes(source)
            source_manifest = json.loads(
                (source / "manifest.json").read_text(encoding="utf-8"))
            source_manifest_hash = sha256_file(source / "manifest.json")
            output = root / "converted"

            result = convert_v2_bundle(source, output)

            self.assertEqual(source_before, self.tree_hashes(source))
            self.assertEqual(result["schema_version"], 3)
            self.assertEqual(result["profile"], "visual")
            validation = validate_bundle(output, "legacy-01", "visual")
            provenance = validation["provenance"]["v2_conversion"]
            self.assertEqual(provenance["source_schema_version"], 2)
            self.assertEqual(provenance["source_manifest_sha256"], source_manifest_hash)
            self.assertEqual(
                provenance["source_asset_sha256"],
                {
                    filename: source_manifest["assets"][filename]["sha256"]
                    for filename in REQUIRED_ASSETS
                },
            )
            converted_metadata = json.loads(
                (output / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(converted_metadata["source_bag_sha256"], "a" * 64)
            self.assertEqual(
                converted_metadata["conversion_provenance"]["source_manifest_sha256"],
                source_manifest_hash,
            )

    def test_conversion_rejects_output_inside_source(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_v2_bundle(Path(directory))
            with self.assertRaisesRegex(ValueError, "output_must_not_be_inside_source_bundle"):
                convert_v2_bundle(source, source / "converted")


if __name__ == "__main__":
    unittest.main()
