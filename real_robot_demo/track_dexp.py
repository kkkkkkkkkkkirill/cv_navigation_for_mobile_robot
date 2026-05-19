#!/usr/bin/env python3
"""Standalone YOLO11n + OC-SORT person tracker for the DEXP DWC-FHD03 USB camera.

Mirrors the detection/tracking logic of person_tracker_node.py from
amr_stage5_tracking, but reads frames straight from /dev/video0 via OpenCV
and shows the annotated stream in a cv2 window. No ROS, no Gazebo.

Press 'q' (with the window focused) to quit.
"""
import argparse
import sys
import time

import cv2
import numpy as np

PERSON_CLASS = 0


def import_ocsort():
    try:
        from boxmot.trackers.ocsort.ocsort import OcSort
        return OcSort
    except ImportError:
        pass
    try:
        from boxmot import OcSort
        return OcSort
    except ImportError:
        from boxmot import OCSORT
        return OCSORT


def open_camera(device, width, height, fps):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f"Failed to open camera {device}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    actual = (
        int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        cap.get(cv2.CAP_PROP_FPS),
    )
    print(f"[track_dexp] camera open: {actual[0]}x{actual[1]} @ {actual[2]:.1f}fps")
    return cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--weights', default='/home/rover/Desktop/amr_stage5_tracking/yolo11n.pt')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--iou', type=float, default=0.45)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--max-age', type=int, default=30)
    ap.add_argument('--min-hits', type=int, default=3)
    ap.add_argument('--asso-threshold', type=float, default=0.3)
    ap.add_argument('--window', default='DEXP tracking')
    args = ap.parse_args()

    from ultralytics import YOLO
    OcSort = import_ocsort()

    print(f"[track_dexp] loading YOLO weights: {args.weights}")
    detector = YOLO(args.weights)
    tracker = OcSort(
        det_thresh=args.conf,
        max_age=args.max_age,
        min_hits=args.min_hits,
        iou_threshold=args.asso_threshold,
        delta_t=3, inertia=0.2, use_byte=False,
    )

    cap = open_camera(args.device, args.width, args.height, args.fps)
    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)

    last_t = time.time()
    fps_smoothed = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[track_dexp] frame grab failed, retrying...")
                time.sleep(0.01)
                continue

            r = detector(
                frame, classes=[PERSON_CLASS], conf=args.conf, iou=args.iou,
                imgsz=args.imgsz, verbose=False,
            )[0]
            if r.boxes is not None and len(r.boxes) > 0:
                xyxy = r.boxes.xyxy.cpu().numpy()
                cf = r.boxes.conf.cpu().numpy()
                cl = r.boxes.cls.cpu().numpy()
                dets = np.column_stack([xyxy, cf, cl]).astype(np.float32)
            else:
                dets = np.empty((0, 6), dtype=np.float32)

            tracks = tracker.update(dets, frame)

            annotated = frame
            for t in tracks:
                x1, y1, x2, y2 = map(int, t[:4])
                tid = int(t[4])
                tconf = float(t[5]) if len(t) > 5 else 1.0
                color = ((tid * 41) % 255, (tid * 73) % 255, (tid * 113) % 255)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    annotated, f"ID {tid} {tconf:.2f}",
                    (x1, max(15, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
                )

            now = time.time()
            dt = now - last_t
            last_t = now
            if dt > 0:
                inst = 1.0 / dt
                fps_smoothed = 0.9 * fps_smoothed + 0.1 * inst if fps_smoothed else inst

            cv2.putText(
                annotated, f"persons: {len(tracks)}  {fps_smoothed:.1f} fps",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
            )

            cv2.imshow(args.window, annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
