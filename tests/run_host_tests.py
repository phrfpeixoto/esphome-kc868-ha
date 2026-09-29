"""Compile the actual component against minimal ESPHome doubles; no firmware SDK needed."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="kc868-ha-tests-") as tmp:
    include = Path(tmp)
    (include / "esphome.h").write_text((root / "tests/esphome_stubs.h").read_text())
    for name in ("core/component.h", "components/uart/uart.h",
                 "components/binary_sensor/binary_sensor.h", "components/switch/switch.h"):
        header = include / "esphome" / name
        header.parent.mkdir(parents=True, exist_ok=True)
        header.write_text('#include "esphome.h"\n')
    command = [os.environ.get("CXX", "c++"), "-std=c++17"]
    if sys.platform == "darwin":
        sdk = subprocess.check_output(["xcrun", "--show-sdk-path"], text=True).strip()
        command += ["-isystem", str(Path(sdk) / "usr/include/c++/v1")]
    command += ["-I" + tmp, "-I" + str(root / "components/kc868_ha"),
                str(root / "components/kc868_ha/kc868_ha_component.cpp"),
                str(root / "tests/test_kc868_ha.cpp"), "-o", str(include / "test")]
    subprocess.run(command, check=True)
    subprocess.run([str(include / "test")], check=True)
