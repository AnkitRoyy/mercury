#!/usr/bin/env python3
"""
Standalone visualizer for lane_costmap_node detection pipeline.
No ROS needed — reads directly from camera.

Usage:
    python3 lane_costmap_viz.py               # /dev/video0
    python3 lane_costmap_viz.py --device 2    # /dev/video2
    python3 lane_costmap_viz.py --image frame.jpg  # from image file

Keys:
    q       quit
    s       save current frame
    1-5     switch display panel
"""

import cv2
import numpy as np
import math
import argparse

# ── Params (mirror your perception.launch.py values) ──────────────────────────
WHITE_V_MIN     = 130
WHITE_S_MAX     = 80
BLOB_MIN_ASPECT = 2.5
BLOB_MAX_CIRC   = 0.35
BLOB_MIN_AREA   = 60
HOUGH_THRESH    = 15
HOUGH_MIN_LEN   = 25.0
HOUGH_MAX_GAP   = 40.0
MIN_SPAN_FRAC   = 0.03
ROI_TOP_FRAC    = 0.35
LETHAL_PX       = 20
SAMPLE_ROWS     = 8

# Fake costmap grid for visualization (top-down bird's eye)
GRID_SIZE_PX    = 500   # display pixels
GRID_M          = 10.0  # real-world metres shown (10x10m window)
# ──────────────────────────────────────────────────────────────────────────────


def white_mask(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
                       np.array([0, 0, WHITE_V_MIN]),
                       np.array([180, WHITE_S_MAX, 255]))
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, bright = cv2.threshold(gray, WHITE_V_MIN, 255, cv2.THRESH_BINARY)
    return cv2.bitwise_and(mask, bright)


def filter_blobs(mask):
    out = np.zeros_like(mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    kept = rejected_asp = rejected_circ = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < BLOB_MIN_AREA:
            continue
        rect = cv2.minAreaRect(cnt)
        w, h = rect[1]
        if w < 1.0 or h < 1.0:
            continue
        aspect = max(w, h) / min(w, h)
        if aspect < BLOB_MIN_ASPECT:
            rejected_asp += 1
            continue
        perimeter = cv2.arcLength(cnt, closed=True)
        if perimeter < 1.0:
            continue
        circularity = (4.0 * math.pi * area) / (perimeter * perimeter)
        if circularity >= BLOB_MAX_CIRC:
            rejected_circ += 1
            continue
        cv2.drawContours(out, [cnt], -1, 255, cv2.FILLED)
        kept += 1
    return out, kept, rejected_asp, rejected_circ


def detect_lines(frame):
    fh, fw = frame.shape[:2]
    roi_y  = int(fh * ROI_TOP_FRAC)
    roi    = frame[roi_y:fh, :]
    roi_h  = roi.shape[0]
    img_cx = fw / 2.0

    raw_mask   = white_mask(roi)
    filt_mask, kept, rej_asp, rej_circ = filter_blobs(raw_mask)

    k         = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 25))
    closed    = cv2.morphologyEx(filt_mask, cv2.MORPH_CLOSE, k)
    edges     = cv2.Canny(closed, 30, 100)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                             threshold=HOUGH_THRESH,
                             minLineLength=HOUGH_MIN_LEN,
                             maxLineGap=HOUGH_MAX_GAP)

    left_segs, right_segs = [], []
    if lines is not None:
        for ln in lines:
            x1, y1, x2, y2 = ln[0]
            if x2 == x1:
                continue
            slope = abs((y2 - y1) / float(x2 - x1))
            if not (0.2 <= slope <= 4.0):
                continue
            length = float(np.hypot(x2 - x1, y2 - y1))
            if (x1 + x2) / 2.0 < img_cx:
                left_segs.append((x1, y1, x2, y2, length))
            else:
                right_segs.append((x1, y1, x2, y2, length))

    def _fit(segs):
        if not segs:
            return None
        ms, bs, ws, y_vals = [], [], [], []
        for (x1, y1, x2, y2, w) in segs:
            dx = float(x2 - x1)
            if dx == 0:
                continue
            m = (y2 - y1) / dx
            ms.append(m); bs.append(y1 - m * x1); ws.append(w)
            y_vals.extend([y1, y2])
        if not ms:
            return None
        if y_vals and MIN_SPAN_FRAC > 0:
            if (max(y_vals) - min(y_vals)) < MIN_SPAN_FRAC * roi_h:
                return None
        tw   = sum(ws)
        m_av = sum(m * w for m, w in zip(ms, ws)) / tw
        b_av = sum(b * w for b, w in zip(bs, ws)) / tw
        return (m_av, b_av) if abs(m_av) > 1e-6 else None

    return (_fit(left_segs), _fit(right_segs),
            roi_y, roi_h, fw,
            raw_mask, filt_mask, closed, edges,
            left_segs, right_segs,
            kept, rej_asp, rej_circ)


def build_costmap_viz(left_fit, right_fit, roi_y, roi_h, fw, fh):
    """
    Fake top-down costmap: robot at bottom-centre, forward = up.
    Uses the line fits to mark approximate lane walls without real TF.
    """
    grid = np.full((GRID_SIZE_PX, GRID_SIZE_PX, 3), 40, dtype=np.uint8)  # dark grey = unknown
    
    # Draw robot (blue dot at bottom centre)
    robot_px = (GRID_SIZE_PX // 2, int(GRID_SIZE_PX * 0.8))
    cv2.circle(grid, robot_px, 8, (200, 100, 0), -1)
    cv2.putText(grid, 'ROBOT', (robot_px[0] - 25, robot_px[1] + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 100, 0), 1)

    if left_fit is None and right_fit is None:
        cv2.putText(grid, 'NO LANES DETECTED', (60, GRID_SIZE_PX // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return grid

    # Project line samples into the fake top-down view
    # Map: image bottom = near robot, image top (roi_y) = far
    # x offset from centre → lateral position
    step = max(1, roi_h // SAMPLE_ROWS)
    roi_sample_end = int(roi_h * 0.5)
    m_per_px_lateral = (GRID_M / GRID_SIZE_PX)  # rough scale

    for y_roi in range(0, roi_sample_end, step):
        # Fraction along ROI from bottom (0=near, 1=far)
        frac = y_roi / max(1, roi_sample_end)
        # Map to grid: near = bottom, far = above robot
        grid_y = int(robot_px[1] - frac * GRID_SIZE_PX * 0.6)

        lx = rx = None
        if left_fit:
            m, b = left_fit
            lx = float(np.clip((y_roi - b) / m, 0.0, fw - 1.0))
        if right_fit:
            m, b = right_fit
            rx = float(np.clip((y_roi - b) / m, 0.0, fw - 1.0))

        # Mark lethal (red) and free (green) bands
        if lx is not None:
            off_norm = (lx - fw / 2.0) / (fw / 2.0)
            for du in range(0, LETHAL_PX, 4):
                x_norm = ((lx - du) - fw / 2.0) / (fw / 2.0)
                gx = int(GRID_SIZE_PX // 2 + x_norm * GRID_SIZE_PX * 0.4)
                if 0 <= gx < GRID_SIZE_PX and 0 <= grid_y < GRID_SIZE_PX:
                    cv2.circle(grid, (gx, grid_y), 3, (0, 0, 200), -1)

        if rx is not None:
            for du in range(0, LETHAL_PX, 4):
                x_norm = ((rx + du) - fw / 2.0) / (fw / 2.0)
                gx = int(GRID_SIZE_PX // 2 + x_norm * GRID_SIZE_PX * 0.4)
                if 0 <= gx < GRID_SIZE_PX and 0 <= grid_y < GRID_SIZE_PX:
                    cv2.circle(grid, (gx, grid_y), 3, (0, 0, 200), -1)

        # Free space inside
        inner_lx = (lx + LETHAL_PX) if lx is not None else max(0.0, (rx or fw/2) - 400.0)
        inner_rx = (rx - LETHAL_PX) if rx is not None else min(float(fw), (lx or fw/2) + 400.0)
        for u in np.arange(inner_lx, inner_rx, 20.0):
            x_norm = (u - fw / 2.0) / (fw / 2.0)
            gx = int(GRID_SIZE_PX // 2 + x_norm * GRID_SIZE_PX * 0.4)
            if 0 <= gx < GRID_SIZE_PX and 0 <= grid_y < GRID_SIZE_PX:
                cv2.circle(grid, (gx, grid_y), 2, (0, 150, 0), -1)

    # Legend
    cv2.rectangle(grid, (5, 5), (170, 70), (60, 60, 60), -1)
    cv2.circle(grid, (20, 20), 5, (0, 0, 200), -1);  cv2.putText(grid, 'LETHAL (wall)', (30, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 200), 1)
    cv2.circle(grid, (20, 40), 5, (0, 150, 0), -1);  cv2.putText(grid, 'FREE (inside)', (30, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 150, 0), 1)
    cv2.circle(grid, (20, 60), 5, (200, 100, 0), -1); cv2.putText(grid, 'ROBOT', (30, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 100, 0), 1)

    return grid


def draw_debug(frame, left_fit, right_fit, roi_y, roi_h, fw,
               raw_mask, filt_mask, closed, edges,
               left_segs, right_segs, kept, rej_asp, rej_circ):
    fh = frame.shape[0]
    debug = frame.copy()

    # ROI border
    cv2.rectangle(debug, (0, roi_y), (fw, fh), (0, 80, 0), 2)

    # Hough segments
    for (x1, y1, x2, y2, _) in left_segs:
        cv2.line(debug, (x1, y1 + roi_y), (x2, y2 + roi_y), (255, 100, 0), 2)
    for (x1, y1, x2, y2, _) in right_segs:
        cv2.line(debug, (x1, y1 + roi_y), (x2, y2 + roi_y), (0, 100, 255), 2)

    # Fitted lines extended
    for fit, col in [(left_fit, (255, 200, 0)), (right_fit, (0, 200, 255))]:
        if fit:
            m, b = fit
            if abs(m) > 1e-6:
                y_top = 0
                y_bot = roi_h - 1
                x_top = int(np.clip((y_top - b) / m, 0, fw - 1))
                x_bot = int(np.clip((y_bot - b) / m, 0, fw - 1))
                cv2.line(debug, (x_top, y_top + roi_y), (x_bot, y_bot + roi_y), col, 3)

    # HUD
    status = []
    if left_fit:  status.append('L:OK')
    else:         status.append('L:--')
    if right_fit: status.append('R:OK')
    else:         status.append('R:--')

    det_col = (0, 255, 0) if (left_fit or right_fit) else (0, 0, 255)
    cv2.putText(debug, f'{" ".join(status)}  blobs: kept={kept} rej_asp={rej_asp} rej_circ={rej_circ}',
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, det_col, 2)
    cv2.putText(debug, 'BLUE=left hough  ORANGE=right hough  CYAN/YELLOW=fitted',
                (10, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    return debug


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', type=int, default=0)
    ap.add_argument('--image', type=str, default=None)
    args = ap.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f'Cannot read {args.image}'); return
        cap = None
    else:
        cap = cv2.VideoCapture(args.device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not cap.isOpened():
            print(f'Cannot open /dev/video{args.device}'); return
        frame = None

    panel = 1
    save_count = 0
    print('Keys: q=quit  s=save  1=debug  2=white_mask  3=blob_filter  4=edges  5=costmap')

    while True:
        if cap is not None:
            ret, frame = cap.read()
            if not ret:
                break

        result = detect_lines(frame)
        (left_fit, right_fit, roi_y, roi_h, fw,
         raw_mask, filt_mask, closed, edges,
         left_segs, right_segs, kept, rej_asp, rej_circ) = result

        debug     = draw_debug(frame, left_fit, right_fit, roi_y, roi_h, fw,
                                raw_mask, filt_mask, closed, edges,
                                left_segs, right_segs, kept, rej_asp, rej_circ)
        costmap   = build_costmap_viz(left_fit, right_fit, roi_y, roi_h, fw, frame.shape[0])

        fh = frame.shape[0]
        panels = {
            1: debug,
            2: cv2.cvtColor(cv2.resize(raw_mask,  (fw, fh)), cv2.COLOR_GRAY2BGR),
            3: cv2.cvtColor(cv2.resize(filt_mask, (fw, fh)), cv2.COLOR_GRAY2BGR),
            4: cv2.cvtColor(cv2.resize(edges,     (fw, fh)), cv2.COLOR_GRAY2BGR),
            5: cv2.resize(costmap, (fw, fh)),
        }
        labels = {1:'1:debug', 2:'2:white_mask', 3:'3:blob_filter', 4:'4:canny_edges', 5:'5:costmap_topdown'}

        display = panels[panel].copy()
        cv2.putText(display, f'[{labels[panel]}]  press 1-5 to switch',
                    (10, display.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 255), 1)

        # Also show a small costmap thumbnail in corner when not on panel 5
        if panel != 5:
            thumb = cv2.resize(costmap, (160, 160))
            display[10:170, fw-170:fw-10] = thumb
            cv2.putText(display, 'costmap', (fw - 165, 185),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 255), 1)

        cv2.imshow('Lane Costmap Viz', display)

        if args.image:
            cv2.waitKey(0)
            break

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('s'):
            fname = f'lane_save_{save_count:03d}.jpg'
            cv2.imwrite(fname, display)
            print(f'Saved {fname}')
            save_count += 1
        elif k in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
            panel = int(chr(k))

    if cap:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()