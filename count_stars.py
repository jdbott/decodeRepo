#!/usr/bin/env python3
"""Count stars in a JPEG image by finding local-maximum point sources against the sky.

Uses local background subtraction (a heavily-blurred copy of the image) so smooth
gradients like Milky Way glow don't get misread as "bright", then finds individual
local maxima rather than connected blobs — this keeps stars that are close together
(e.g. in a dense galactic core) from being merged and undercounted as one star.

Usage:
    python3 count_stars.py path/to/image.jpg [--sigma 4] [--min-dist 3] [--show]
"""

import argparse
import sys

import cv2
import numpy as np


def count_stars(
    image_path,
    sigma_mult=4.0,
    min_dist=3,
    exclude_bottom=0.0,
    show=False,
):
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)

    # Estimate the smooth local background (galactic glow, light-pollution gradient)
    # and keep only pixels that stand out above it.
    background = cv2.GaussianBlur(gray, (0, 0), sigmaX=25)
    diff = gray - background
    threshold = diff.std() * sigma_mult
    bright_mask = diff > threshold

    if exclude_bottom > 0:
        h = gray.shape[0]
        cutoff = int(h * (1 - exclude_bottom))
        bright_mask[cutoff:, :] = False

    # Local maxima: a pixel that equals the max of its neighborhood is a peak.
    # This separates stars that sit close together instead of merging them into
    # one connected-component blob.
    kernel_size = 2 * min_dist + 1
    dilated = cv2.dilate(diff, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)))
    peak_mask = (diff == dilated) & bright_mask

    ys, xs = np.nonzero(peak_mask)
    star_centroids = list(zip(xs, ys))

    if show:
        annotated = img.copy()
        for x, y in star_centroids:
            cv2.circle(annotated, (int(x), int(y)), 6, (0, 0, 255), 1)
        cv2.imshow("Detected stars", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return len(star_centroids), star_centroids


def main():
    parser = argparse.ArgumentParser(description="Count stars in a JPEG image.")
    parser.add_argument("image", help="Path to the JPEG image")
    parser.add_argument(
        "--sigma", type=float, default=4.0, dest="sigma_mult",
        help="Sensitivity: pixel must be this many std-devs above local background (default: 4, lower = more/fainter detections)",
    )
    parser.add_argument(
        "--min-dist", type=int, default=3,
        help="Minimum pixel separation between two distinct star peaks (default: 3)",
    )
    parser.add_argument(
        "--exclude-bottom", type=float, default=0.0,
        help="Fraction of the image height to exclude from the bottom, e.g. 0.2 to mask out a treeline/horizon (default: 0)",
    )
    parser.add_argument("--show", action="store_true", help="Display the image with detected stars circled")
    args = parser.parse_args()

    try:
        count, _ = count_stars(
            args.image,
            sigma_mult=args.sigma_mult,
            min_dist=args.min_dist,
            exclude_bottom=args.exclude_bottom,
            show=args.show,
        )
    except FileNotFoundError as e:
        print(e, file=sys.stderr)
        sys.exit(1)

    print(f"Stars detected: {count}")


if __name__ == "__main__":
    main()
