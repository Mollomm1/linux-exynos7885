# SPDX-License-Identifier: GPL-2.0-only
"""Host-only refusal-path checks; never contact hardware."""

import argparse
import importlib.util
import json
from pathlib import Path
import tempfile
import tarfile
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location("scsc", Path(__file__).with_name("scsc-dev.py"))
scsc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scsc)


class SafetyChecks(unittest.TestCase):
    def test_capture_preserves_complete_tree_and_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            tree = Path(directory) / "tree"
            tree.mkdir()
            config = "CONFIG_MODULES=y\nCONFIG_MODULE_UNLOAD=y\nCONFIG_EXYNOS_SCSC_WIFIBT=m\n"
            files = {".config": config, "Module.symvers": "symbols\n",
                     "include/config/kernel.release": "release\n",
                     "include/generated/utsversion.h": '#define UTS_VERSION "version"\n',
                     "arch/arm64/boot/Image": "kernel",
                     "arch/arm64/boot/dts/exynos/exynos7904-t510.dtb": "dtb",
                     "source-file.c": "retained source"}
            for name, content in files.items():
                path = tree / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
            archive = Path(directory) / "baseline.tar.gz"
            args = argparse.Namespace(baseline=tree, archive=archive, commit="test",
                                      cc="gcc", cross_compile="")
            with patch.object(scsc, "compiler", return_value="test compiler"):
                scsc.capture(args)
            data = json.loads((tree / scsc.MANIFEST).read_text())
            self.assertEqual(data["version"], "version")
            self.assertEqual(data["files"], scsc.fingerprints(tree))
            with tarfile.open(archive) as saved:
                self.assertEqual(saved.extractfile("./source-file.c").read(), b"retained source")
            with self.assertRaisesRegex(RuntimeError, "already captured"):
                scsc.capture(args)

    def test_ssh_target_cannot_inject_options_or_shell(self):
        for target in ("-oProxyCommand=id", "root@host;id", "user@host\nid", "$(id)"):
            with self.assertRaises(RuntimeError):
                scsc.Remote(target)

    def test_kernel_mismatch_stops_before_manifest_read(self):
        remote = Mock()
        remote.shell.return_value = "wrong-release\nversion\naarch64\nsamsung,t510"
        with self.assertRaisesRegex(RuntimeError, "kernel"):
            scsc.identity(remote, {"release": "expected", "version": "version"})
        self.assertEqual(remote.shell.call_count, 1)
        remote.root.assert_not_called()

    def test_other_board_is_rejected(self):
        remote = Mock()
        remote.shell.return_value = "release\nversion\naarch64\nsamsung,other"
        with self.assertRaisesRegex(RuntimeError, "T510"):
            scsc.identity(remote, {"release": "release", "version": "version"})

    def test_manifest_mismatch_is_rejected(self):
        remote = Mock()
        remote.shell.side_effect = ["release\nversion\naarch64\nsamsung,t510", "{}"]
        with self.assertRaisesRegex(RuntimeError, "manifest"):
            scsc.identity(remote, {"release": "release", "version": "version"})

    def test_corrupt_module_never_opens_ssh(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            (out / "module.json").write_text(json.dumps({"sha256": "wrong"}))
            (out / (scsc.MODULE + ".ko")).write_bytes(b"changed")
            with patch.object(scsc, "Remote") as remote:
                with self.assertRaisesRegex(RuntimeError, "checksum"):
                    scsc.hardware(argparse.Namespace(out=out))
                remote.assert_not_called()

    def test_changed_baseline_stops_before_make(self):
        with tempfile.TemporaryDirectory() as directory:
            tree = Path(directory)
            (tree / scsc.MANIFEST).write_text(json.dumps({"files": {"expected": "hash"}}))
            with patch.object(scsc, "fingerprints", return_value={}), patch.object(scsc, "run") as run:
                with self.assertRaisesRegex(RuntimeError, "artifacts"):
                    scsc.build(argparse.Namespace(baseline=tree))
                run.assert_not_called()

    def test_changed_compiler_stops_before_make(self):
        with tempfile.TemporaryDirectory() as directory:
            tree = Path(directory)
            data = {"files": {}, "cc": "gcc", "compiler": "old"}
            (tree / scsc.MANIFEST).write_text(json.dumps(data))
            with patch.object(scsc, "fingerprints", return_value={}), \
                    patch.object(scsc, "compiler", return_value="new"), \
                    patch.object(scsc, "run") as run:
                with self.assertRaisesRegex(RuntimeError, "Compiler"):
                    scsc.build(argparse.Namespace(baseline=tree))
                run.assert_not_called()

    def test_cycle_stops_after_first_warning(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            ko = out / (scsc.MODULE + ".ko")
            ko.write_bytes(b"test")
            (out / "module.json").write_text(json.dumps({"baseline": {}, "sha256": scsc.digest(ko)}))
            remote = Mock()
            remote.root.side_effect = ["WARNING: old fault", "", "", "WARNING: new fault", "WARNING: new fault"]
            with patch.object(scsc, "Remote", return_value=remote), patch.object(scsc, "identity"):
                with self.assertRaisesRegex(RuntimeError, "warning"):
                    scsc.hardware(argparse.Namespace(out=out, target="user@172.16.42.1", action="cycle"))
            commands = [call.args[0] for call in remote.root.call_args_list]
            self.assertEqual(sum("insmod " in command for command in commands), 1)


if __name__ == "__main__":
    unittest.main()
