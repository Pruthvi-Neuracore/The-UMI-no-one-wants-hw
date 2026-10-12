"""Copy chopstick/README.md to the repository root, with its relative links pointed into chopstick/.

    python sync_readme.py
"""
import re

from parts import ROOT

LOCAL = ("docs/", "hardware/", "assembly/", "urdf/", "retarget/", "cad/", "NOTICE.md", "LICENSE")


def main():
    text = (ROOT / "README.md").read_text()

    def fix(m):
        attr, path = m.group(1), m.group(2)
        return f"{attr}chopstick/{path}" if path.startswith(LOCAL) else m.group(0)

    text = re.sub(r'(src="|\]\()([^)"#][^)"]*)', fix, text)
    text = text.replace("](#", "](#")                 # in-page anchors stay as they are
    out = ROOT.parent / "README.md"
    out.write_text(text)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
