#!/usr/bin/env python3
"""[R]overTech -- Lesson 3 -- the "companion computer" of our exercise.

Same order as the slides:
  CAMERA     -> grab webcam frames (slide 7)
  INTRINSICS -> K from the horizontal field of view, no calibration (slide 13)
  FEATURES   -> detect the ArUco marker corners (slide 25)
  PnP        -> 4 known 3D corners + 4 pixels -> metric pose (slide 26)
  LINK       -> MAVLink DEBUG_VECT "ARUCO" to PX4 SITL (slide 32)

Standalone:  python aruco_companion.py --no-px4
With PX4:    python aruco_companion.py --px4 udpout:127.0.0.1:14580
Press q in the video window to quit.
"""
import argparse
import math
import time

import cv2
import numpy as np

MAVLINK_NAME = "ARUCO"  # must match lesson3::kMarkerName in marker_lab.cpp


def parse_args():
    p = argparse.ArgumentParser(description="Webcam -> ArUco -> solvePnP -> distance (-> PX4).")
    p.add_argument("--camera", type=int, default=0, help="webcam index for cv2.VideoCapture")
    p.add_argument("--dict", default="DICT_4X4_50", help="ArUco dictionary (workshop: DICT_4X4_50)")
    p.add_argument("--id", type=int, default=0, help="marker id to track (workshop: 0)")
    p.add_argument("--size", type=float, default=0.05,
                   help="side of the BLACK square in metres (workshop: 0.05 = 5 cm)")
    p.add_argument("--hfov", type=float, default=70.0,
                   help="webcam horizontal field of view in degrees (look it up, then tune it)")
    p.add_argument("--px4", default="udpout:127.0.0.1:14580",
                   help="pymavlink connection string of PX4 SITL (onboard link)")
    p.add_argument("--no-px4", action="store_true", help="do not send anything to PX4")
    return p.parse_args()


def intrinsics_from_fov(width, height, hfov_deg):
    """Slide 13: fx = (W/2) / tan(HFOV/2). Square pixels, centred principal point."""
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    K = np.array([[fx, 0.0, width / 2.0],
                  [0.0, fx, height / 2.0],
                  [0.0, 0.0, 1.0]])
    dist = np.zeros(5)  # no distortion model: a known bias (slide 15)
    return K, dist


def marker_object_points(size):
    """Marker corners in the MARKER frame [m], in OpenCV's order TL, TR, BR, BL."""
    h = size / 2.0
    return np.array([[-h, h, 0.0],
                     [h, h, 0.0],
                     [h, -h, 0.0],
                     [-h, -h, 0.0]])


class Px4Link:
    """Tiny MAVLink sender: we behave like an onboard (companion) computer."""

    def __init__(self, url):
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.conn = mavutil.mavlink_connection(
            url, source_system=1,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
        self.t0 = time.monotonic()
        self.last_heartbeat = 0.0

    def heartbeat(self):
        now = time.monotonic()
        if now - self.last_heartbeat >= 1.0:
            self.conn.mav.heartbeat_send(self.mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                         self.mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat = now

    def send_marker(self, t_cam):
        # x, y, z = marker position in the CAMERA optical frame [m].
        # PX4 republishes it on uORB debug_vect; marker_lab rotates it to FRD.
        time_usec = int((time.monotonic() - self.t0) * 1e6)
        self.conn.mav.debug_vect_send(MAVLINK_NAME.encode(), time_usec,
                                      float(t_cam[0]), float(t_cam[1]), float(t_cam[2]))


def main():
    args = parse_args()

    cap = cv2.VideoCapture(args.camera)
    ok, frame = cap.read()
    if not ok:
        raise SystemExit(f"cannot read from webcam {args.camera}")
    height, width = frame.shape[:2]

    K, dist = intrinsics_from_fov(width, height, args.hfov)
    fx = K[0, 0]
    print(f"image {width}x{height} px, HFOV {args.hfov:.1f} deg -> fx = fy = {fx:.1f} px")
    print("K =\n" + np.array2string(K, precision=1, suppress_small=True))

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict))
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    obj_pts = marker_object_points(args.size)

    px4 = None if args.no_px4 else Px4Link(args.px4)
    if px4:
        print(f"sending DEBUG_VECT '{MAVLINK_NAME}' to {args.px4}")

    last_print = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if px4:
            px4.heartbeat()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        text = f"marker id {args.id} not found"
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for c, marker_id in zip(corners, ids.flatten()):
                if marker_id != args.id:
                    continue
                img_pts = c.reshape(4, 2).astype(np.float64)

                # PnP: 4 known 3D points + their 4 pixels -> marker pose in the camera.
                ok_pnp, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist,
                                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok_pnp:
                    continue
                t = tvec.flatten()                 # [x right, y down, z forward] m
                distance = float(np.linalg.norm(t))

                # Pinhole check (slide 11): Z ~ fx * L / side_px for a facing marker.
                side_px = np.mean(np.linalg.norm(img_pts - np.roll(img_pts, 1, axis=0), axis=1))
                z_pinhole = fx * args.size / side_px

                cv2.drawFrameAxes(frame, K, dist, rvec, tvec, args.size / 2)
                text = f"dist {distance:.2f} m | Z {t[2]:.2f} m | pinhole Z {z_pinhole:.2f} m"

                if px4:
                    px4.send_marker(t)

                now = time.monotonic()
                if now - last_print >= 0.5:
                    print(f"t_cam = [{t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}] m | {text}")
                    last_print = now
                break

        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Lesson 3 - ArUco companion (q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
