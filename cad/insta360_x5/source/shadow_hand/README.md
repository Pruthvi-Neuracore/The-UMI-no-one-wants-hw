# Shadow Hand (renders only)

The right Shadow Hand from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/shadow_hand)
(Apache-2.0, see `LICENSE`), posed half-relaxed and exported in its forearm frame (mm) by `export.py`:

    python export.py mujoco_menagerie/shadow_hand/right_hand.xml shadow_hand.npz   # needs mujoco

then split into `shadow_forearm.ply`, `shadow_hand.ply` and `shadow_metal.ply` for colouring. Used by `hand_robot.png`.
