#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Build and manually exercise the inert T510 module against a frozen kernel."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time

MODULE = "exynos-scsc-wifibt"
MANIFEST = "scsc-baseline.json"
DRIVER = "drivers/soc/samsung/exynos-scsc-wifibt.c"
STATE = "/sys/bus/platform/devices/120c0000.wifibt/state"


def run(argv, **kwargs):
    return subprocess.run(argv, check=True, **kwargs)


def output(argv):
    return subprocess.check_output(argv, text=True).strip()


def digest(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def compiler(cc):
    return output([cc, "--version"])


def fingerprints(tree):
    names = [".config", "Module.symvers", "include/config/kernel.release",
             "arch/arm64/boot/Image", "arch/arm64/boot/dts/exynos/exynos7904-t510.dtb"]
    for directory in ("include/generated", "arch/arm64/include/generated"):
        names += [str(p.relative_to(tree)) for p in (tree / directory).rglob("*")
                  if p.is_file()]
    return {name: digest(tree / name) for name in sorted(names)}


def capture(args):
    tree = args.baseline.resolve()
    require(not (tree / "source").exists(), "Capture requires the aport's in-tree build")
    require(not (tree / MANIFEST).exists(), "Baseline already captured; do not overwrite")
    config = (tree / ".config").read_text()
    for setting in ("CONFIG_MODULES=y", "CONFIG_MODULE_UNLOAD=y",
                    "CONFIG_EXYNOS_SCSC_WIFIBT=m"):
        require(setting in config.splitlines(), "Missing " + setting)
    require("CONFIG_LOCALVERSION_AUTO=y" not in config, "Disable LOCALVERSION_AUTO")
    uts = (tree / "include/generated/utsversion.h").read_text()
    version = re.search(r'#define UTS_VERSION "([^"]+)"', uts).group(1)
    data = {"format": 1, "commit": args.commit,
            "release": (tree / "include/config/kernel.release").read_text().strip(),
            "version": version, "cc": args.cc, "cross_compile": args.cross_compile,
            "compiler": compiler(args.cc), "files": fingerprints(tree)}
    data["id"] = hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()
    (tree / MANIFEST).write_text(json.dumps(data, indent=2) + "\n")
    if shutil.which("apk"):
        (tree / "scsc-build-packages.txt").write_text(output(["apk", "info", "-v"]) + "\n")
    archive = args.archive.resolve()
    require(tree != archive.parent and tree not in archive.parents,
            "Archive must be outside the build tree")
    require(not archive.exists(), "Archive already exists; retain it or choose another name")
    temporary = archive.with_name(archive.name + ".partial")
    run(["tar", "-czf", str(temporary), "-C", str(tree), "."])
    temporary.replace(archive)
    print(f"Frozen baseline {data['id']} in {archive}")


def build(args):
    tree = args.baseline.resolve()
    data = json.loads((tree / MANIFEST).read_text())
    require(fingerprints(tree) == data["files"], "Baseline artifacts have changed")
    require(compiler(data["cc"]) == data["compiler"], "Compiler differs from baseline")
    out = args.out.resolve()
    require(not out.exists(), "Use a new output directory for each iteration")
    require(tree not in out.parents, "Module output must be outside the frozen tree")
    out.mkdir(parents=True)
    shutil.copy2(args.source.resolve() / DRIVER, out / (MODULE + ".c"))
    (out / "Makefile").write_text(f"obj-m := {MODULE}.o\n")
    run(["make", "-C", str(tree), "ARCH=arm64", "CC=" + data["cc"],
         "CROSS_COMPILE=" + data["cross_compile"], "M=" + str(out), "modules"])
    require(fingerprints(tree) == data["files"], "Build modified baseline artifacts")
    ko = out / (MODULE + ".ko")
    vermagic = output(["modinfo", "-F", "vermagic", str(ko)])
    require(vermagic.split()[0] == data["release"], "Module release differs from baseline")
    require(not output(["modinfo", "-F", "alias", str(ko)]), "Unexpected module autoload alias")
    bundle = {"baseline": data, "sha256": digest(ko), "vermagic": vermagic,
              "source_sha256": digest(out / (MODULE + ".c"))}
    (out / "module.json").write_text(json.dumps(bundle, indent=2) + "\n")
    print(f"Built {ko}; upload does not load it")


def pmb_build(args):
    """Use the retained native chroot compiler; never rebuild the kernel."""
    archive = args.archive.resolve()
    require(archive.parent.name == "cache_distfiles",
            "Pass the archive in pmbootstrap's cache_distfiles directory")
    require(not args.out.exists(), "Use a new output directory")
    identifier = digest(archive)
    shared = archive.parent / "scsc-dev" / identifier

    def inside(path):
        return "/var/cache/distfiles/" + str(path.relative_to(archive.parent))

    pmb = ["pmbootstrap", "-w", str(archive.parent.parent), "chroot", "--"]
    # Distfiles is owned by abuild, not by the host login user. Let the
    # existing pmbootstrap privilege path create just our staging directory.
    if not shared.exists():
        run(pmb + ["install", "-d", "-m", "0755", "-o", str(os.getuid()),
                   "-g", str(os.getgid()), inside(shared)])
    tree = shared / "baseline"
    if not tree.exists():
        temporary = shared / "extracting"
        require(not temporary.exists(), "Incomplete extraction: inspect the extracting directory")
        temporary.mkdir()
        run(["tar", "--no-same-owner", "-xzf", str(archive), "-C", str(temporary)])
        temporary.rename(tree)
    iteration = shared / str(time.time_ns())
    source = iteration / "source"
    (source / DRIVER).parent.mkdir(parents=True)
    shutil.copy2(args.source / DRIVER, source / DRIVER)
    script = iteration / "scsc-dev.py"
    shutil.copy2(Path(__file__), script)

    run(pmb + ["python3",
         inside(script), "build", "--baseline", inside(tree),
         "--source", inside(source), "--out", inside(iteration / "out")])
    args.out.mkdir(parents=True)
    for name in (MODULE + ".ko", "module.json"):
        shutil.copy2(iteration / "out" / name, args.out / name)
    print(f"Candidate bundle: {args.out.resolve()}")


class Remote:
    def __init__(self, target):
        require(re.fullmatch(r"(?:[a-zA-Z0-9_][a-zA-Z0-9_.-]*@)?[a-zA-Z0-9][a-zA-Z0-9.-]*", target),
                "Use user@hostname or user@IPv4")
        self.target = target
        self.ssh = ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
                    "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=5",
                    "-o", "ServerAliveCountMax=2", target]

    def shell(self, script):
        result = run(self.ssh + ["sh -s"], input="set -eu\n" + script,
                     text=True, stdout=subprocess.PIPE, timeout=30)
        return result.stdout.strip()

    def root(self, script):
        wrapper = "if [ \"$(id -u)\" = 0 ]; then sh -c {0}; " \
                  "elif command -v sudo >/dev/null; then sudo -n sh -c {0}; " \
                  "else doas -n sh -c {0}; fi"
        return self.shell(wrapper.format(shlex.quote("set -eu\n" + script)))


def identity(remote, data):
    info = remote.shell("uname -r; uname -v; uname -m; tr '\\000' '\\n' "
                        "< /proc/device-tree/compatible").splitlines()
    require(info[:3] == [data["release"], data["version"], "aarch64"],
            "Installed kernel does not match the frozen baseline")
    require("samsung,t510" in info[3:], "Target is not the T510")
    installed = json.loads(remote.shell("cat /usr/share/scsc-dev/baseline.json"))
    require(installed == data, "Installed baseline manifest differs")


def hardware(args):
    bundle = json.loads((args.out / "module.json").read_text())
    ko = args.out / (MODULE + ".ko")
    require(digest(ko) == bundle["sha256"], "Module checksum differs from manifest")
    remote = Remote(args.target)
    identity(remote, bundle["baseline"])
    directory = "/tmp/scsc-dev-" + bundle["sha256"]
    module_path = directory + "/" + MODULE + ".ko"
    quoted_state = shlex.quote(STATE)

    def status():
        return remote.root(f"uname -a; cat /proc/modules; "
                           f"if test -f {quoted_state}; then cat {quoted_state}; fi; "
                           "dmesg | tail -150")

    def unload():
        remote.root(f"test \"$(cat {quoted_state})\" = inert; rmmod {MODULE}; "
                    "test ! -d /sys/module/exynos_scsc_wifibt")

    def load():
        remote.root("test ! -d /sys/module/exynos_scsc_wifibt; "
                    f"cd {shlex.quote(directory)}; sha256sum -c module.sha256; "
                    f"insmod {shlex.quote(module_path)} enable=1; "
                    f"test \"$(cat {quoted_state})\" = inert")

    logdir = args.out / "logs"
    logdir.mkdir(exist_ok=True)
    stamp = str(time.time_ns()) + "-" + args.action
    before = status()
    (logdir / (stamp + "-before.txt")).write_text(before + "\n")
    try:
        if args.action == "upload":
            # Root-owned destination; do not use a user-writable module path.
            remote.root(f"mkdir -m 700 {shlex.quote(directory)}")
            # Stream bytes through SSH, with elevation for the destination.
            command = f"umask 077; cat > {shlex.quote(module_path)}"
            wrapper = ("if [ \"$(id -u)\" = 0 ]; then sh -c {0}; "
                       "elif command -v sudo >/dev/null; then sudo -n sh -c {0}; "
                       "else doas -n sh -c {0}; fi").format(shlex.quote(command))
            with ko.open("rb") as stream:
                run(remote.ssh + [wrapper], stdin=stream, timeout=30)
            checksum = bundle["sha256"] + "  " + MODULE + ".ko\n"
            remote.root(f"cd {shlex.quote(directory)}; "
                        f"printf %s {shlex.quote(checksum)} > module.sha256; "
                        "sha256sum -c module.sha256")
            print("Uploaded only; use load explicitly")
        elif args.action == "load":
            load()
        elif args.action == "unload":
            unload()
        elif args.action == "cycle":
            for number in range(10):
                start = time.monotonic()
                load()
                unload()
                print(f"Inert cycle {number + 1}/10: {time.monotonic() - start:.3f}s including SSH")
                current = status()
                (logdir / f"{stamp}-cycle-{number + 1}.txt").write_text(current + "\n")
                # Stop on new warning signatures; never clear the kernel log.
                signatures = r"WARNING:|BUG:|Oops:|Call trace:|Kernel panic|watchdog:.*lockup"
                require(re.findall(signatures, current) == re.findall(signatures, before),
                        "Kernel warning detected; stop and inspect logs")
        elif args.action == "status":
            print(before)
    finally:
        try:
            (logdir / (stamp + "-after.txt")).write_text(status() + "\n")
        except (subprocess.SubprocessError, RuntimeError) as error:
            print(f"Cannot collect final log: {error}; stop testing", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest="command", required=True)
    cap = subs.add_parser("capture", help="Used by APKBUILD after the baseline build")
    cap.add_argument("--baseline", type=Path, required=True)
    cap.add_argument("--archive", type=Path, required=True)
    cap.add_argument("--commit", required=True)
    cap.add_argument("--cc", required=True)
    cap.add_argument("--cross-compile", default="")
    cap.set_defaults(func=capture)
    make = subs.add_parser("build", help="Run inside the original pmbootstrap compiler environment")
    make.add_argument("--baseline", type=Path, required=True)
    make.add_argument("--source", type=Path, required=True)
    make.add_argument("--out", type=Path, required=True)
    make.set_defaults(func=build)
    pmb = subs.add_parser("pmb-build", help="Host wrapper for module-only builds in pmbootstrap")
    pmb.add_argument("--archive", type=Path, required=True)
    pmb.add_argument("--source", type=Path, required=True)
    pmb.add_argument("--out", type=Path, required=True)
    pmb.set_defaults(func=pmb_build)
    for action in ("upload", "load", "unload", "cycle", "status"):
        sub = subs.add_parser(action)
        sub.add_argument("--target", default="user@172.16.42.1")
        sub.add_argument("--out", type=Path, required=True)
        sub.set_defaults(func=hardware, action=action)
    args = parser.parse_args()
    try:
        args.func(args)
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        parser.exit(1, f"Stopped: {error}\n")


if __name__ == "__main__":
    main()
