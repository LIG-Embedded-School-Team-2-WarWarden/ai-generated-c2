# LiDAR detector source

`target_detector.h` is copied without changes from the team's LiDAR repository:
https://github.com/LIG-Embedded-School-Team-2-WarWarden/LiDAR

Pinned commit: `6aecfb5f7491718b554e2482110910085ddc2b12` (2026-10-19).
File blob: `bee763c0ab01eef7840a55789dc29175438f4a24`.
No license file is present at that revision; no new license is asserted here.
Only `Watch::Detector` is reused. Upstream BoxTracker and prediction are not used.
C2's separate single-object follow loop repeatedly commands device angles from fresh detections.

Input is the full organised SDK CAMERA XYZ array in metres and original depth in mm.
Invalid slots retain their pixel index. Filtering and object centroids are computed in C2.
