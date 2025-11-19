import sys
from pathlib import Path

from .decoders import register_decoders


def main():
    register_decoders()

    # `bag_to_lerobot`` is not released yet so we have to run it from source.
    current_file_dir = Path(__file__).parent.resolve()
    ws_src_dir = current_file_dir.parent.parent.parent.resolve()

    if str(ws_src_dir) not in sys.path:
        sys.path.insert(0, str(ws_src_dir))

    from iblnkn.rosetta.scripts import bag_to_lerobot

    bag_to_lerobot.main()


if __name__ == "__main__":
    main()
