import sys
from pathlib import Path

from .decoders import register_decoders


def main():
    register_decoders()

    from .upstream.bag_to_lerobot import main as bag_to_lerobot_main

    bag_to_lerobot_main()


if __name__ == "__main__":
    main()
