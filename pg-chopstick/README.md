# PG Chopstick: build code

The generators for the PG Chopstick kits (Python, build123d). Their output goes to `pg-chopstick/<kit>/` and
`pg-chopstick/docs/media/`, the same layout as on `main`, where the printable files and docs live.

```
cad/insta360_x5/   cad/d405/    part scripts, kinematics, URDF, assembly, renders, the HandUMI T-gripper (t_gripper.py)
cad/make_video.py               the overview video and GIF
```

See `pg-chopstick/README.md` on `main` for the build order and the Python packages. Meta-owned reference meshes are
not committed; `cad/<kit>/source/*/README.md` says where to put them.
