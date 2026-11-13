#!/usr/bin/env python3
"""Interactive/headless viewer for the offline buoy detector prototypes.

No ROS topics are published and no production source is modified.  Detection
overlays come only from the algorithms in buoy_offline_diagnostic.py.
"""

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from buoy_offline_diagnostic import Candidate, experimental_candidates, production


WINDOW = "Dagozilla offline buoy detector"
COLORS = {"red": (0, 0, 255), "green": (0, 220, 0),
          "unknown": (0, 210, 255)}


@dataclass
class Detection:
    candidate: Candidate
    status: str
    label: str
    track_id: int = -1
    object_score: float = 0.0
    color_score: float = 0.0
    reason: str = ""


class ViewerTrack:
    def __init__(self, track_id, item, frame_index):
        self.id = track_id
        self.x, self.y, self.radius = item.x, item.y, item.radius
        self.vx = self.vy = 0.0
        self.color_scores = {"red": 0.0, "green": 0.0}
        self.hits, self.misses, self.last_frame = 0, 0, frame_index
        self.update(item, frame_index)

    def affinity(self, item):
        px, py = self.x + self.vx, self.y + self.vy
        distance = math.hypot(item.x - px, item.y - py)
        position = max(0.0, 1.0 - distance / max(22.0, 3.0*self.radius))
        size = min(self.radius, item.radius) / max(1.0, self.radius, item.radius)
        return .72*position + .28*size

    def update(self, item, frame_index):
        old_x, old_y = self.x, self.y
        self.x = .68*item.x + .32*self.x
        self.y = .68*item.y + .32*self.y
        self.radius = .65*item.radius + .35*self.radius
        self.vx = .65*self.vx + .35*(self.x-old_x)
        self.vy = .65*self.vy + .35*(self.y-old_y)
        for color in self.color_scores:
            self.color_scores[color] *= .90
        if item.color in self.color_scores:
            self.color_scores[item.color] += item.color_score
        self.hits += 1
        self.misses = 0
        self.last_frame = frame_index

    def color(self):
        red, green = self.color_scores["red"], self.color_scores["green"]
        if max(red, green) < .55 or abs(red-green) < .22:
            return "unknown"
        return "red" if red > green else "green"


class ExperimentalTracker:
    """Small nearest-candidate tracker for visualization, not production."""

    def __init__(self):
        self.tracks = []
        self.next_id = 1

    def reset(self):
        self.tracks.clear()
        self.next_id = 1

    @staticmethod
    def proposal_score(item):
        # Shape and colour are soft evidence. Neither is a hard buoy condition.
        return float(np.clip(.58*item.shape_score + .42*item.color_score, 0, 1))

    def update(self, frame_index, proposals):
        used, detections = set(), []
        for track in self.tracks:
            best_index, best_affinity = -1, 0.0
            for index, item in enumerate(proposals):
                if index in used:
                    continue
                affinity = track.affinity(item)
                if affinity > best_affinity:
                    best_index, best_affinity = index, affinity
            if best_index >= 0 and best_affinity >= .42:
                item = proposals[best_index]
                used.add(best_index)
                base = self.proposal_score(item)
                object_score = .72*base + .28*best_affinity
                track.update(item, frame_index)
                color = track.color()
                if track.hits >= 3 and object_score >= .58:
                    status = "CONFIRMED"
                    label = f"{color.upper()} BUOY" if color != "unknown" else "UNKNOWN BUOY"
                elif track.hits >= 2 and object_score >= .46:
                    status, label = "TRACKED", "TRACKED CANDIDATE"
                else:
                    status, label = "WEAK", "WEAK CANDIDATE"
                detections.append(Detection(item, status, label, track.id,
                                            object_score, item.color_score))
            else:
                track.misses += 1
                track.x += track.vx
                track.y += track.vy
                if track.misses <= 5 and track.hits >= 3:
                    color = track.color()
                    ghost = Candidate("temporal", color, track.x, track.y,
                        track.radius, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                    detections.append(Detection(
                        ghost, "TRACKED", "TRACKED UNKNOWN COLOR", track.id,
                        max(.30, .68-.08*track.misses), 0.0,
                        f"NO MATCH {track.misses}/5"))
        self.tracks = [track for track in self.tracks if track.misses <= 5]

        for index, item in enumerate(proposals):
            if index in used:
                continue
            score = self.proposal_score(item)
            # New tracks require reasonable aggregate evidence. This is a
            # candidate threshold, not a hard circularity/colour rule.
            if score < .52 or item.radius < 3:
                continue
            track = ViewerTrack(self.next_id, item, frame_index)
            self.next_id += 1
            self.tracks.append(track)
            detections.append(Detection(item, "WEAK", "WEAK CANDIDATE",
                                        track.id, score, item.color_score))
        return detections


def detector_data(frame, frame_index, tracker):
    hsv, before, after, current = production(frame)
    accepted = [item for item in current if item.accepted]
    rejected = [item for item in current if not item.accepted]
    strict_red = cv2.bitwise_or(before["red_low"], before["red_high"])
    strict = cv2.bitwise_or(strict_red, before["green"])
    morph_red = cv2.bitwise_or(after["red_low"], after["red_high"])
    morph = cv2.bitwise_or(morph_red, after["green"])
    relaxed, joined, edges, proposals = experimental_candidates(
        frame, hsv, strict)
    # The exact production accepts are also valid experimental proposals.
    experimental = tracker.update(frame_index, accepted + proposals)
    current_detections = []
    for item in accepted:
        label = f"{item.color.upper()} BUOY"
        current_detections.append(Detection(
            item, "CONFIRMED", label, -1, item.score, item.color_score))
    return {"before": before, "after": after, "strict_red": strict_red,
            "strict": strict, "morph": morph, "relaxed": relaxed,
            "joined": joined, "edges": edges, "current": current_detections,
            "experimental": experimental, "rejected": rejected}


def draw_multiline(image, lines, origin, color, scale=.46):
    x, y = origin
    height = 18
    width = max(100, max((cv2.getTextSize(line, 0, scale, 1)[0][0]
                         for line in lines), default=100)+8)
    top = max(0, y-height*len(lines)-5)
    cv2.rectangle(image, (max(0,x-3), top),
                  (min(image.shape[1]-1,x+width), y+3), (0,0,0), -1)
    for index, line in enumerate(lines):
        cv2.putText(image, line, (x, top+15+height*index), 0, scale,
                    color, 1, cv2.LINE_AA)


def draw_detections(frame, detections, rejected, show_rejected, show_tracks):
    output = frame.copy()
    for detection in detections:
        item = detection.candidate
        color = COLORS.get(item.color, COLORS["unknown"])
        if detection.status == "WEAK":
            color = (0, 165, 255)
        elif detection.status == "TRACKED":
            color = (255, 220, 0)
        center = (round(item.x), round(item.y))
        radius = max(3, round(item.radius))
        cv2.circle(output, center, radius, color, 2)
        x, y = center[0]-radius, center[1]-radius
        cv2.rectangle(output, (x,y), (center[0]+radius,center[1]+radius), color, 1)
        lines = [f"{detection.label} [{detection.status}]",
                 f"area={item.area:.0f} radius={item.radius:.1f}",
                 f"object={detection.object_score:.2f} color={detection.color_score:.2f}"]
        if show_tracks and detection.track_id >= 0:
            lines.insert(1, f"track={detection.track_id}")
        if detection.reason:
            lines.append(detection.reason)
        draw_multiline(output, lines, (max(2,x), max(58,y)), color)
    if show_rejected:
        # Only show meaningful production contours, not every tiny noise speck.
        for item in rejected:
            if item.area < 35:
                continue
            center=(round(item.x),round(item.y)); radius=max(2,round(item.radius))
            cv2.circle(output,center,radius,(150,150,150),1)
            reason = "AREA<300" if item.area < 300 else "REJECTED"
            cv2.putText(output, f"REJECTED {reason} A={item.area:.0f}",
                        (max(0,center[0]-radius),max(14,center[1]-radius)),
                        0,.36,(190,190,190),1,cv2.LINE_AA)
    return output


def title(image, text):
    cv2.rectangle(image,(0,0),(image.shape[1],27),(0,0,0),-1)
    cv2.putText(image,text,(7,19),0,.52,(255,255,255),1,cv2.LINE_AA)
    return image


def mask_view(data):
    red=cv2.cvtColor(data["strict_red"],cv2.COLOR_GRAY2BGR)
    green=cv2.cvtColor(data["before"]["green"],cv2.COLOR_GRAY2BGR)
    title(red,"STRICT RED COMBINED"); title(green,"STRICT GREEN")
    return np.hstack([red,green])


def stages_view(frame, data, final):
    views=[frame,cv2.cvtColor(data["strict"],cv2.COLOR_GRAY2BGR),
           cv2.cvtColor(data["relaxed"],cv2.COLOR_GRAY2BGR),
           cv2.cvtColor(data["morph"],cv2.COLOR_GRAY2BGR),final]
    labels=["ORIGINAL","STRICT MASK","RELAXED MASK","CURRENT MORPHOLOGY","FINAL DETECTIONS"]
    cells=[]
    for view,label in zip(views,labels):
        cell=cv2.resize(view,(320,240)); title(cell,label); cells.append(cell)
    blank=np.zeros_like(cells[0]); title(blank,"LEGEND: D rejected | T tracks | M view")
    return np.vstack([np.hstack(cells[:3]),np.hstack(cells[3:]+[blank])])


def add_hud(image, frame_index, fps, processing_fps, detector, view, paused):
    line=(f"frame={frame_index} time={frame_index/fps:.3f}s "
          f"proc={processing_fps:.1f}FPS detector={detector} view={view}"
          + (" PAUSED" if paused else ""))
    cv2.rectangle(image,(0,image.shape[0]-25),(image.shape[1],image.shape[0]),(0,0,0),-1)
    cv2.putText(image,line,(7,image.shape[0]-7),0,.48,(0,255,255),1,cv2.LINE_AA)


def render(frame, data, detector, view, show_rejected, show_tracks):
    current=draw_detections(frame,data["current"],data["rejected"],show_rejected,False)
    experimental=draw_detections(frame,data["experimental"],data["rejected"],show_rejected,show_tracks)
    if detector == "compare":
        title(current,"CURRENT"); title(experimental,"EXPERIMENTAL")
        overlay=np.hstack([current,experimental])
    else:
        overlay=current if detector == "current" else experimental
        title(overlay,detector.upper())
    if view == "mask":
        return mask_view(data)
    if view == "stages":
        final=experimental if detector != "current" else current
        return stages_view(frame,data,final)
    return overlay


def parse_args():
    parser=argparse.ArgumentParser()
    parser.add_argument("--video",type=Path,required=True)
    parser.add_argument("--show",action="store_true")
    parser.add_argument("--output",type=Path)
    parser.add_argument("--detector",choices=("current","experimental","compare"),default="compare")
    parser.add_argument("--view",choices=("overlay","mask","stages"),default="overlay")
    parser.add_argument("--start-frame",type=int)
    parser.add_argument("--end-frame",type=int)
    parser.add_argument("--start-time",type=float)
    parser.add_argument("--end-time",type=float)
    parser.add_argument("--debug-rejected",action="store_true")
    parser.add_argument("--hide-tracks",action="store_true")
    return parser.parse_args()


def main():
    args=parse_args()
    if not args.show and args.output is None:
        raise SystemExit("headless mode requires --output")
    capture=cv2.VideoCapture(str(args.video))
    if not capture.isOpened(): raise SystemExit(f"cannot open {args.video}")
    fps=capture.get(cv2.CAP_PROP_FPS) or 30.0
    total=int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    start=args.start_frame if args.start_frame is not None else round((args.start_time or 0)*fps)
    end=args.end_frame if args.end_frame is not None else total-1
    if args.end_time is not None: end=min(end,round(args.end_time*fps))
    start=max(0,min(start,total-1)); end=max(start,min(end,total-1))
    capture.set(cv2.CAP_PROP_POS_FRAMES,start)
    tracker=ExperimentalTracker(); writer=None; paused=False; step=False
    show_rejected=args.debug_rejected; show_tracks=not args.hide_tracks
    view=args.view; index=start; processing_fps=0.0
    last_display=None; last_processed_index=None; last_written_index=None
    while index <= end:
        if not paused or step or last_display is None:
            capture.set(cv2.CAP_PROP_POS_FRAMES,index)
            ok,frame=capture.read()
            if not ok: break
            begin=time.perf_counter(); data=detector_data(frame,index,tracker)
            display=render(frame,data,args.detector,view,show_rejected,show_tracks)
            elapsed=max(1e-6,time.perf_counter()-begin)
            instant=1.0/elapsed
            processing_fps=instant if processing_fps==0 else .85*processing_fps+.15*instant
            add_hud(display,index,fps,processing_fps,args.detector,view,paused)
            last_display=display
            last_processed_index=index
            if args.output and index != last_written_index:
                if writer is None:
                    args.output.parent.mkdir(parents=True,exist_ok=True)
                    writer=cv2.VideoWriter(str(args.output),cv2.VideoWriter_fourcc(*"mp4v"),fps,
                                           (display.shape[1],display.shape[0]))
                    if not writer.isOpened(): raise SystemExit(f"cannot write {args.output}")
                writer.write(display)
                last_written_index=index
            if not paused: index += 1
            step=False
        if not args.show:
            continue
        cv2.imshow(WINDOW,last_display)
        delay=1 if not paused else 30
        key=cv2.waitKeyEx(delay)
        if key in (27,ord('q'),ord('Q')): break
        if key==ord(' '): paused=not paused
        elif key in (ord('d'),ord('D')): show_rejected=not show_rejected; step=True
        elif key in (ord('t'),ord('T')): show_tracks=not show_tracks; step=True
        elif key in (ord('m'),ord('M')):
            view={"overlay":"mask","mask":"stages","stages":"overlay"}[view]; step=True
        elif paused and key in (ord('n'),ord('N'),2555904,65363):
            index=min(end,(last_processed_index if last_processed_index is not None else index)+1)
            step=True
        elif paused and key in (ord('b'),ord('B'),2424832,65361):
            index=max(start,(last_processed_index if last_processed_index is not None else index)-1)
            tracker.reset(); step=True
    capture.release()
    if writer: writer.release()
    if args.show: cv2.destroyAllWindows()
    print(f"processed frames {start}..{min(index,end)} from {args.video}")
    if args.output: print(f"wrote {args.output}")


if __name__ == "__main__": main()
