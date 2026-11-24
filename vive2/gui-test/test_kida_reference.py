#!/usr/bin/env python3
"""Compare C++ FK against installed pytact and the source YAML.

Run with ../../.venv/bin/python test_kida_reference.py after ./build.sh --test.
Pure kinematics: does not start an environment or controller.
"""
import ast
import io
from pathlib import Path
import subprocess
import numpy as np
import tact

HERE = Path(__file__).resolve().parent
KIDA = HERE.parent.parent
model = tact.Model(str(KIDA / "yaml/kida"))
rng = np.random.default_rng(19)
qs = rng.uniform(-1.4, 1.4, (100, 14))
qs[0] = 0
qs[1] = [-.4, .2, 0, 1.7, .2, .3, 0]*2
source = "\n".join(" ".join(map(str, q)) for q in qs)
output = subprocess.run([str(HERE / "test_arm_ik"), "--fk"], input=source,
                        text=True, capture_output=True, check=True)
actual = np.loadtxt(io.StringIO(output.stdout)).reshape(-1, 2, 4, 4)
expected = np.array([model.fkh(["tcp1", "tcp2"], q) for q in qs])
error = np.max(np.abs(actual-expected))
assert error < 1e-10, f"C++ / pytact FK mismatch: {error}"
print(f"PASS: 100 dual-arm poses match installed pytact FK (max matrix error {error:.3g})")

# Check the visual hand's orientation against the actual runner's hand mounts,
# independently of the C++ implementation. Procedural axes are +X=fingers,
# +Z=dorsum; the hand YAML uses +Z=fingers and -X=dorsum.
mounts = {}
for call in ast.walk(ast.parse((KIDA / "kida-run").read_text())):
    if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Attribute) or call.func.attr != "add":
        continue
    keywords = {kw.arg: kw.value for kw in call.keywords}
    if "base" not in keywords or "offset" not in keywords:
        continue
    base = ast.literal_eval(keywords["base"])
    if base in ("tcp1", "tcp2"):
        angles = [ast.literal_eval(n) for n in keywords["offset"].elts[3:]]
        mounts[base] = tact.euler_to_rotation(angles, eulerseq="XYZ", deg=True)
assert len(mounts) == 2
output = subprocess.run([str(HERE / "test_arm_ik"), "--hand-frames"], input=source,
                        text=True, capture_output=True, check=True)
actual_hands = np.loadtxt(io.StringIO(output.stdout)).reshape(-1, 2, 4, 4)
expected_hands = expected.copy()
visual_to_yaml = tact.euler_to_rotation([0, -90, 0], eulerseq="XYZ", deg=True)
for side, base in enumerate(("tcp1", "tcp2")):
    expected_hands[:, side, :3, :3] = expected[:, side, :3, :3] @ mounts[base] @ visual_to_yaml
error = np.max(np.abs(actual_hands-expected_hands))
assert error < 1e-10, f"Visual hand / kida-run mount mismatch: {error}"
for side, sign in enumerate((1, -1)):
    np.testing.assert_allclose(actual_hands[0, side, :3, 2], [0, sign, 0], atol=1e-10)
    np.testing.assert_allclose(actual_hands[0, side, :3, 0], [0, 0, -1], atol=1e-10)
print(f"PASS: 100 dual-arm visual hand orientations match kida-run mounts; zero-pose dorsums face outward (max error {error:.3g})")
