# Quest Touch Pro controller (renders, checks and the collar's bore)

`quest.py` stands `MetaQuestTouchPro_Right.stl` upright above the D405, facing forward, and measures its handle's
outline over the collar's height into `handle_section.json` (an outline only), which `quest_arm` is bored to. Meta's
model is not redistributed: put the STL in this folder to re-measure or to render. Without it the cached outline is used
for the part and a capsule stand-in for renders.
