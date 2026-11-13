#!/usr/bin/env python3
"""Extract an evenly sampled, timestamped contact sheet from a video."""

import argparse
import math
from pathlib import Path

import cv2


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--samples", type=int, default=48)
    parser.add_argument("--columns", type=int, default=6)
    parser.add_argument("--thumb-width", type=int, default=320)
    args = parser.parse_args()

    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise SystemExit(f"cannot open {args.video}")
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = capture.get(cv2.CAP_PROP_FPS)
    indices = [round(i * (frame_count - 1) / max(1, args.samples - 1))
               for i in range(args.samples)]
    thumbs = []
    for index in indices:
        capture.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = capture.read()
        if not ok:
            continue
        scale = args.thumb_width / frame.shape[1]
        thumb = cv2.resize(frame, None, fx=scale, fy=scale)
        cv2.rectangle(thumb, (0, 0), (thumb.shape[1], 25), (0, 0, 0), -1)
        cv2.putText(thumb, f"f={index} t={index / fps:.1f}s", (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    cv2.LINE_AA)
        thumbs.append(thumb)
    capture.release()

    rows = math.ceil(len(thumbs) / args.columns)
    cell_h = max(image.shape[0] for image in thumbs)
    sheet = 255 * __import__("numpy").ones(
        (rows * cell_h, args.columns * args.thumb_width, 3), dtype="uint8")
    for i, thumb in enumerate(thumbs):
        row, col = divmod(i, args.columns)
        sheet[row * cell_h:row * cell_h + thumb.shape[0],
              col * args.thumb_width:(col + 1) * args.thumb_width] = thumb
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), sheet):
        raise SystemExit(f"cannot write {args.output}")
    print(f"wrote {args.output}: {len(thumbs)} samples, {frame_count} frames, {fps:.3f} fps")


if __name__ == "__main__":
    main()
