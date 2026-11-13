#!/usr/bin/env python3
"""Batch extract frames from video files for dataset labeling and training."""

import argparse
from pathlib import Path
import cv2


def batch_video_to_images(input_folder: str, output_folder: str, skip: int = 10):
    input_path = Path(input_folder)
    out_path = Path(output_folder)

    if not input_path.exists():
        print(f"Error: Input directory '{input_path}' does not exist.")
        return

    out_path.mkdir(parents=True, exist_ok=True)
    video_extensions = {".mp4", ".avi", ".mkv", ".mov", ".webm"}

    video_files = sorted(
        f for f in input_path.iterdir() if f.is_file() and f.suffix.lower() in video_extensions
    )

    if not video_files:
        print(f"No video files found in '{input_path}'.")
        return

    print(f"Processing {len(video_files)} video(s) from '{input_path}' to '{out_path}' (skip={skip})...")

    total_saved = 0
    for video_file in video_files:
        cap = cv2.VideoCapture(str(video_file))
        if not cap.isOpened():
            print(f"Error: Could not open video {video_file.name}")
            continue

        count = 0
        saved = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if count % skip == 0:
                file_name = f"{video_file.stem}_frame_{saved:04d}.jpg"
                cv2.imwrite(str(out_path / file_name), frame)
                saved += 1
            count += 1

        cap.release()
        total_saved += saved
        print(f"{video_file.name}: read {count} frames, saved {saved} frames")

    print(f"Extraction complete! Total saved frames: {total_saved}")


def main():
    parser = argparse.ArgumentParser(description="Batch extract frames from video files.")
    parser.add_argument(
        "--input", "-i",
        type=str,
        default="videos",
        help="Path to folder containing videos (default: 'videos')",
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default="output_frames",
        help="Path to folder to save extracted frames (default: 'output_frames')",
    )
    parser.add_argument(
        "--skip", "-s",
        type=int,
        default=10,
        help="Extract 1 frame every N frames (default: 10)",
    )
    args = parser.parse_args()

    batch_video_to_images(args.input, args.output, args.skip)


if __name__ == "__main__":
    main()
