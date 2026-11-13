import cv2
import os
from pathlib import Path

def batch_video_to_images(input_folder, output_folder, skip=10):
    # Using your hardcoded paths
    input_path = Path('/home/bertrand/Videos/video/')
    out_path = Path('/home/bertrand/Downloads/Bola_new/full')
    
    # Create the single output directory once
    out_path.mkdir(parents=True, exist_ok=True)

    video_extensions = {'.mp4', '.avi', '.mkv', '.mov', '.webm'}

    for video_file in sorted(input_path.iterdir()):
        if video_file.suffix.lower() not in video_extensions:
            continue

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
                # Add video_file.stem to the filename to prevent overwriting frames
                file_name = f"{video_file.stem}_frame_{saved:04d}.jpg"
                cv2.imwrite(str(out_path / file_name), frame)
                saved += 1
            count += 1

        cap.release()
        print(f"{video_file.name}: read {count} frames, saved {saved}")

batch_video_to_images('videos/', 'output_frames/', skip=10)