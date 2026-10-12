# Notice

The chopstick gripper is from [TToTMooN/KIWI_hardware](https://github.com/TToTMooN/KIWI_hardware) at commit `626ad7b`
(11 Oct 2026). It is licensed under Apache-2.0; see `LICENSE`. KIWI is by Benjamin C. Yang, Weiying Wang, Shenggao Li,
Keming Yan, Sasha Wilkinson, Zelin Wang, Yip Fun Yeung and Lingfeng Sun:

```bibtex
@article{yang2026kiwi,
  title   = {Kinematic Interface for the Wild: Modular Bimanual Loco-Manipulation Capture from 360° Cameras Alone},
  author  = {Yang, Benjamin C. and Wang, Weiying and Li, Shenggao and Yan, Keming and Wilkinson, Sasha and
             Wang, Zelin and Yeung, Yip Fun and Sun, Lingfeng},
  journal = {arXiv preprint arXiv:2609.22809},
  year    = {2026}
}
```

Unmodified copies from KIWI_hardware:

- `cad/source/kiwi/chop_DC.step` and `chop_SA.step` (from `chopsticks/assets/`)
- `hardware/print/KIWI-3dprint.3mf`
- `hardware/tags/*.pdf`

Changes in this repository:

- `hardware/STEP/*.step` and `hardware/STL/*.stl` are the KIWI parts split out of the two assemblies by
  `cad/extract_parts.py` and renamed. The jaws and tips are moved to the closed pose (the robot-gripper driver jaw is
  turned back 44.21° about its pivot). The geometry is otherwise unchanged. `x5_button_cap` and `x5_button_shaft` come
  from the 3MF.
- `openarm_adapter` is new. It replaces KIWI's `plate_openarm`, which does not match the OpenArm link7 flange used here.
- The kinematics JSON, the URDFs and their meshes, the assemblies, the renders and the `retarget/` package are new.
  They are derived from the geometry above.
- The Insta360 X5 and servo bodies in the URDFs and assemblies are KIWI's simplified envelopes, not vendor models.

The OpenArm link7 geometry used to fit and check the adapter, and the arm in the renders, come from the OpenArm
description in the Neuracore robots repository (`example_openarm/openarm_description`). Those meshes are not
redistributed here.
