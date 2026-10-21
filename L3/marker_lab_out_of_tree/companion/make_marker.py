#!/usr/bin/env python3
"""[R]overTech -- Lesson 3 -- generate a printable ArUco marker.

    python make_marker.py --id 0 --dict DICT_4X4_50

Print the PNG at 100% scale (no "fit to page"), then MEASURE the side of
the black square with a ruler: that number, in metres, is --size for
aruco_companion.py. The white border is not part of the marker size.
"""
import argparse

import cv2


def main():
    parser = argparse.ArgumentParser(description="Generate a printable ArUco marker PNG.")
    parser.add_argument("--id", type=int, default=0, help="marker id (TBD by the workshop)")
    parser.add_argument("--dict", default="DICT_4X4_50", help="ArUco dictionary, e.g. DICT_4X4_50")
    parser.add_argument("--px", type=int, default=800, help="side of the black square in pixels")
    parser.add_argument("--border", type=int, default=100, help="white quiet zone in pixels")
    args = parser.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict))
    marker = cv2.aruco.generateImageMarker(dictionary, args.id, args.px)

    # The detector needs a white quiet zone around the black border.
    marker = cv2.copyMakeBorder(marker, args.border, args.border, args.border, args.border,
                                cv2.BORDER_CONSTANT, value=255)

    out = f"marker_{args.dict}_{args.id}.png"
    cv2.imwrite(out, marker)
    print(f"saved {out}")
    print("Print at 100% scale and measure the BLACK side in metres -> --size")


if __name__ == "__main__":
    main()
