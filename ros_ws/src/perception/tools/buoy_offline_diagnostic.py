#!/usr/bin/env python3
"""Offline audit/benchmark for the production and experimental buoy detectors.

This never publishes ROS commands and does not replace the production node.
"""

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np


TARGETS = {
    "red_low": ((0, 120, 70), (10, 255, 255), "red"),
    "red_high": ((170, 120, 70), (179, 255, 255), "red"),
    "green": ((35, 80, 60), (85, 255, 255), "green"),
}


@dataclass
class Candidate:
    source: str
    color: str
    x: float
    y: float
    radius: float
    area: float
    bbox_w: int
    bbox_h: int
    aspect: float
    circularity: float
    solidity: float
    extent: float
    ellipse_ratio: float
    color_score: float
    shape_score: float
    temporal_score: float = 0.0
    score: float = 0.0
    accepted: bool = False
    reason: str = ""


def features(contour, source, color, evidence_mask):
    area = float(cv2.contourArea(contour))
    perimeter = float(cv2.arcLength(contour, True))
    x, y, w, h = cv2.boundingRect(contour)
    (cx, cy), radius = cv2.minEnclosingCircle(contour)
    hull_area = max(1.0, float(cv2.contourArea(cv2.convexHull(contour))))
    circularity = 4.0 * math.pi * area / max(1.0, perimeter * perimeter)
    solidity = area / hull_area
    extent = area / max(1.0, w * h)
    ellipse_ratio = 0.0
    if len(contour) >= 5:
        _, axes, _ = cv2.fitEllipse(contour)
        ellipse_ratio = min(axes) / max(1.0, max(axes))
    region = np.zeros(evidence_mask.shape, np.uint8)
    cv2.drawContours(region, [contour], -1, 255, -1)
    pixels = max(1, cv2.countNonZero(region))
    color_score = cv2.countNonZero(cv2.bitwise_and(evidence_mask, region)) / pixels
    shape_score = float(np.clip(
        0.30 * np.clip(circularity, 0, 1) +
        0.25 * np.clip(solidity, 0, 1) +
        0.20 * np.clip(extent / 0.78, 0, 1) +
        0.25 * np.clip(ellipse_ratio, 0, 1), 0, 1))
    return Candidate(source, color, cx, cy, radius, area, w, h, w / max(1, h),
                     circularity, solidity, extent, ellipse_ratio,
                     color_score, shape_score)


def production(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks_before, masks_after, candidates = {}, {}, []
    for name, (lower, upper, color) in TARGETS.items():
        before = cv2.inRange(hsv, np.array(lower), np.array(upper))
        after = cv2.erode(before, None, iterations=2)
        after = cv2.dilate(after, None, iterations=2)
        masks_before[name], masks_after[name] = before, after
        contours, _ = cv2.findContours(after, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            item = features(contour, name, color, before)
            item.accepted = item.area >= 300.0
            item.reason = "accepted" if item.accepted else f"area {item.area:.1f} < min_area 300.0"
            if item.accepted:
                circle_area = max(1.0, math.pi * item.radius * item.radius)
                item.score = min(1.0, item.area / circle_area)
            candidates.append(item)
    return hsv, masks_before, masks_after, candidates


def experimental_candidates(frame, hsv, strong_mask):
    relaxed_red = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 35, 45), (18, 255, 255)),
        cv2.inRange(hsv, (160, 35, 45), (179, 255, 255)))
    relaxed_green = cv2.inRange(hsv, (25, 30, 40), (100, 255, 255))
    relaxed = cv2.bitwise_or(relaxed_red, relaxed_green)
    # Join coloured shell fragments around a specular core without declaring
    # white pixels to be a colour.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    joined = cv2.morphologyEx(relaxed, cv2.MORPH_CLOSE, kernel, iterations=2)
    joined = cv2.morphologyEx(joined, cv2.MORPH_OPEN,
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    contours, _ = cv2.findContours(joined, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    output = []
    for contour in contours:
        if not 45 <= cv2.contourArea(contour) <= frame.shape[0] * frame.shape[1] * .12:
            continue
        item = features(contour, "relaxed_color", "unknown", strong_mask)
        region = np.zeros(joined.shape, np.uint8)
        cv2.drawContours(region, [contour], -1, 255, -1)
        red_fraction = cv2.countNonZero(cv2.bitwise_and(relaxed_red, region)) / max(1, cv2.countNonZero(region))
        green_fraction = cv2.countNonZero(cv2.bitwise_and(relaxed_green, region)) / max(1, cv2.countNonZero(region))
        if max(red_fraction, green_fraction) >= 0.18:
            item.color = "red" if red_fraction > green_fraction else "green"
        item.color_score = max(red_fraction, green_fraction)
        output.append(item)

    # Independent shape proposals. Edges nominate objects; colour is scored
    # afterward and is intentionally not a hard existence condition.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 55, 145)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    edge_contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
    for contour in edge_contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        if not 55 <= area <= 12000 or w < 5 or h < 5 or w / h < .35 or w / h > 2.8:
            continue
        item = features(contour, "shape", "unknown", strong_mask)
        if item.shape_score >= .64:
            output.append(item)
    return relaxed, joined, edges, output


class Track:
    def __init__(self, candidate, frame_index):
        self.x, self.y, self.radius = candidate.x, candidate.y, candidate.radius
        self.vx = self.vy = 0.0
        self.last_frame = frame_index
        self.misses = 0
        self.color_history = {"red": 0.0, "green": 0.0}
        self.update(candidate, frame_index)

    def affinity(self, item):
        distance = math.hypot(item.x - (self.x + self.vx), item.y - (self.y + self.vy))
        position = max(0.0, 1.0 - distance / max(25.0, 3.0 * self.radius))
        size = min(self.radius, item.radius) / max(1.0, self.radius, item.radius)
        return .7 * position + .3 * size

    def update(self, item, frame_index):
        old_x, old_y = self.x, self.y
        self.x = .65 * item.x + .35 * self.x
        self.y = .65 * item.y + .35 * self.y
        self.radius = .6 * item.radius + .4 * self.radius
        self.vx = .7 * self.vx + .3 * (self.x - old_x)
        self.vy = .7 * self.vy + .3 * (self.y - old_y)
        self.last_frame, self.misses = frame_index, 0
        if item.color in self.color_history:
            for key in self.color_history:
                self.color_history[key] *= .92
            self.color_history[item.color] += item.color_score

    def label(self):
        red, green = self.color_history["red"], self.color_history["green"]
        if max(red, green) < .6 or abs(red - green) < .25:
            return "unknown"
        return "red" if red > green else "green"


def panel(frame, masks_before, masks_after, candidates, exp, frame_index, fps):
    red_before = cv2.bitwise_or(masks_before["red_low"], masks_before["red_high"])
    red_after = cv2.bitwise_or(masks_after["red_low"], masks_after["red_high"])
    views = [frame, cv2.cvtColor(masks_before["red_low"], cv2.COLOR_GRAY2BGR),
             cv2.cvtColor(masks_before["red_high"], cv2.COLOR_GRAY2BGR),
             cv2.cvtColor(red_before, cv2.COLOR_GRAY2BGR),
             cv2.cvtColor(masks_before["green"], cv2.COLOR_GRAY2BGR),
             cv2.cvtColor(red_after, cv2.COLOR_GRAY2BGR),
             cv2.cvtColor(masks_after["green"], cv2.COLOR_GRAY2BGR), frame.copy()]
    labels = ["original", "red_low pre", "red_high pre", "red combined pre",
              "green pre", "red post morph", "green post morph", "contours"]
    for item in candidates:
        colour = (0, 255, 0) if item.accepted else (0, 0, 255)
        cv2.circle(views[-1], (round(item.x), round(item.y)), round(item.radius), colour, 2)
        cv2.putText(views[-1], f"{item.source} A={item.area:.0f} {item.reason}",
                    (max(0, round(item.x-item.radius)), max(15, round(item.y-item.radius))),
                    cv2.FONT_HERSHEY_SIMPLEX, .38, colour, 1, cv2.LINE_AA)
    cells = []
    for view, label in zip(views, labels):
        small = cv2.resize(view, (320, 240))
        cv2.rectangle(small, (0, 0), (320, 24), (0, 0, 0), -1)
        cv2.putText(small, label, (5, 17), 0, .48, (255,255,255), 1)
        cells.append(small)
    sheet = np.vstack([np.hstack(cells[:4]), np.hstack(cells[4:])])
    cv2.putText(sheet, f"frame={frame_index} time={frame_index/fps:.3f}s", (5, 475),
                0, .5, (0,255,255), 1)
    return sheet


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--visualize-frames", default="")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    selected = {int(v) for v in args.visualize_frames.split(",") if v}
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened(): raise SystemExit(f"cannot open {args.video}")
    fps = cap.get(cv2.CAP_PROP_FPS); frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    rows, rejects, tracks = [], [], []
    stats = {"frames": 0, "current_red": 0, "current_green": 0,
             "prototype_red": 0, "prototype_green": 0, "prototype_unknown": 0,
             "current_any": 0, "prototype_any": 0}
    index = 0
    while True:
        ok, frame = cap.read()
        if not ok: break
        hsv, before, after, current = production(frame)
        accepted = [c for c in current if c.accepted]
        strong = cv2.bitwise_or(before["green"], cv2.bitwise_or(before["red_low"], before["red_high"]))
        relaxed, joined, edges, proposals = experimental_candidates(frame, hsv, strong)
        pool = accepted + proposals
        used = set(); emitted = []
        for track in tracks:
            best_i, best_aff = -1, 0.0
            for i, item in enumerate(pool):
                if i in used: continue
                affinity = track.affinity(item)
                if affinity > best_aff: best_i, best_aff = i, affinity
            if best_i >= 0 and best_aff >= .42:
                item = pool[best_i]; used.add(best_i); item.temporal_score = best_aff
                item.score = .42*item.shape_score + .33*item.color_score + .25*best_aff
                track.update(item, index)
                if item.score >= .42 or item in accepted:
                    emitted.append((track, item))
            else:
                track.misses += 1; track.x += track.vx; track.y += track.vy
                if track.misses <= 6:
                    ghost = Candidate("temporal", track.label(), track.x, track.y, track.radius,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, max(0,.8-track.misses*.1), True,
                        f"temporal persistence miss={track.misses}")
                    emitted.append((track, ghost))
        tracks = [t for t in tracks if t.misses <= 6]
        for i, item in enumerate(pool):
            if i in used: continue
            item.score = .58*item.shape_score + .42*item.color_score
            if item.score >= .58 and item.radius >= 3:
                track = Track(item, index); tracks.append(track); emitted.append((track,item))

        current_colors = {c.color for c in accepted}
        prototype_colors = {track.label() for track, _ in emitted}
        stats["frames"] += 1
        if accepted: stats["current_any"] += 1
        if emitted: stats["prototype_any"] += 1
        for color in ("red", "green"):
            if color in current_colors: stats[f"current_{color}"] += 1
            if color in prototype_colors: stats[f"prototype_{color}"] += 1
        if "unknown" in prototype_colors: stats["prototype_unknown"] += 1
        rows.append({"frame":index,"time_sec":index/fps,"current_count":len(accepted),
                     "current_colors":"|".join(sorted(current_colors)),
                     "prototype_count":len(emitted),"prototype_colors":"|".join(sorted(prototype_colors))})
        for n, item in enumerate(current):
            if not item.accepted:
                record=asdict(item); record.update(frame=index,time_sec=index/fps,contour=n)
                rejects.append(record)
        if index in selected:
            cv2.imwrite(str(args.output/f"frame_{index:06d}_stages.jpg"),
                        panel(frame,before,after,current,(relaxed,joined,edges,proposals),index,fps))
        index += 1
        if index % 500 == 0: print(f"processed {index}/{frame_count}", flush=True)
    cap.release()
    with (args.output/"frame_results.csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
    with (args.output/"rejected_contours.csv").open("w",newline="") as f:
        fields=list(rejects[0].keys()) if rejects else ["frame","reason"]
        writer=csv.DictWriter(f,fieldnames=fields); writer.writeheader(); writer.writerows(rejects)
    stats.update(fps=fps,duration_sec=index/fps)
    (args.output/"summary.json").write_text(json.dumps(stats,indent=2)+"\n")
    print(json.dumps(stats,indent=2))


if __name__ == "__main__": main()
