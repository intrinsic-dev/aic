from math import pi
import argparse

parser = argparse.ArgumentParser(description="Normalize joint positions.")
parser.add_argument("value", type=float, help="The joint position value to normalize.")
args = parser.parse_args()

value: float = args.value
print("{:.3f}".format(value / (4 * pi) + 0.5))
