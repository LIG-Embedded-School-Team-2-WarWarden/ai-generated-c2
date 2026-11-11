"""Live 3D display of the simulated camera's pan and tilt."""
import math
import tkinter as tk
from tkinter import ttk


class FcsView(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent, padding=(16, 12))
        ttk.Label(self, text='FCS 모의 모터', font=('맑은 고딕', 16, 'bold')).pack(anchor='w')
        self.canvas = tk.Canvas(self, background='#eef2f6', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True, pady=8)
        self.angles = ttk.Label(self, font=('Consolas', 11), justify='left')
        self.angles.pack(anchor='w')
        ttk.Label(self, text='회전량은 마지막 EF03을 받은 순간부터 계산합니다.').pack(anchor='w', pady=(4, 0))
        self.last_frame = None

    def update_pose(self, state):
        pan = state['fcs_azimuth_deg']
        tilt = state['fcs_elevation_deg']
        command_pan = state['fcs_target_azimuth_deg']
        command_tilt = state['fcs_target_elevation_deg']
        pan_travel = state['fcs_pan_travel_deg']
        tilt_travel = state['fcs_tilt_travel_deg']
        self.angles.configure(text=(
            '        현재각     명령각     회전량\n'
            f'PAN    {pan:6.1f}°   {command_pan:6.1f}°   {pan_travel:+6.1f}°\n'
            f'TILT   {tilt:6.1f}°   {command_tilt:6.1f}°   {tilt_travel:+6.1f}°'
        ))
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        frame = (width, height, pan, tilt, command_pan, command_tilt)
        if frame != self.last_frame:
            self.draw(*frame)
            self.last_frame = frame

    def draw(self, width, height, pan, tilt, command_pan, command_tilt):
        canvas = self.canvas
        canvas.delete('all')
        scale = min(width / 6.6, height / 4.7)
        yaw, pitch = map(math.radians, (-36, 29))
        pan, tilt, goal_pan, goal_tilt = map(math.radians, (pan-180, tilt, command_pan-180, command_tilt))
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)

        def project(point):
            x, y, z = point
            across = x*cos_yaw-z*sin_yaw
            forward = x*sin_yaw+z*cos_yaw
            return (width*.5+across*scale, height*.56-(y*cos_pitch-forward*sin_pitch)*scale,
                    forward*cos_pitch+y*sin_pitch)

        def line(points, **style):
            canvas.create_line(*(v for point in points for v in project(point)[:2]), **style)

        def arc(start, end, point, **style):
            line((point(start+(end-start)*step/32) for step in range(33)), **style)

        def rotate_pan(point):
            x, y, z = point
            return x*math.cos(pan)+z*math.sin(pan), y, -x*math.sin(pan)+z*math.cos(pan)

        def rotate_camera(point):
            x, y, z = point
            return rotate_pan((x, 1.03+y*math.cos(tilt)+z*math.sin(tilt),
                               -y*math.sin(tilt)+z*math.cos(tilt)))

        for index in range(-3, 4):
            line(((index, -.36, -3), (index, -.36, 3)), fill='#d4dce5')
            line(((-3, -.36, index), (3, -.36, index)), fill='#d4dce5')

        def pan_mark(angle):
            return 2.18*math.sin(angle), -.34, 2.18*math.cos(angle)

        arc(-math.pi/3, math.pi/3, pan_mark, fill='#a3b0be')
        arc(0, pan, pan_mark, fill='#13799a', width=4)
        for angle in range(-60, 61, 30):
            x, y, z = pan_mark(math.radians(angle))
            line(((x*.96, y, z*.96), (x*1.05, y, z*1.05)), fill='#718499')

        polygons = []

        def box(center, size, transform, shades):
            corners = [(x, y, z) for z in (-1, 1) for y in (-1, 1) for x in (-1, 1)]
            vertices = [project(transform(tuple(c + sign*d/2 for c, sign, d in zip(center, corner, size))))
                        for corner in corners]
            faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4),
                     (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
            for indices, color in zip(faces, shades):
                points = [vertices[index] for index in indices]
                depth = sum(point[2] for point in points) / len(points)
                polygons.append((depth, points, color))

        base = ('#65788c', '#7a90a6', '#51677e', '#adc0d2', '#8399ad', '#62798f')
        camera = ('#14617b', '#2d97ad', '#185b76', '#55b5c3', '#2c91ac', '#227b99')
        box((0, -.17, 0), (2.36, .36, 2), lambda point: point, base)
        box((0, .17, 0), (1.72, .33, 1.54), rotate_pan, base)
        for x in (-.67, .67):
            box((x, .66, 0), (.21, .83, .62), rotate_pan, base)
        box((0, 0, .12), (1.1, .64, 1.42), rotate_camera, camera)
        box((0, 0, .87), (.61, .48, .17), rotate_camera, ('#26394b',)*6)
        box((0, 0, .97), (.37, .29, .05), rotate_camera, ('#76cddd',)*6)
        for _, points, color in sorted(polygons, key=lambda face: face[0]):
            canvas.create_polygon(*(v for point in points for v in point[:2]), fill=color, outline='#3d6174')

        def tilt_mark(angle):
            return rotate_pan((1.02, 1.03+1.45*math.sin(angle), 1.45*math.cos(angle)))

        arc(0, math.pi/3, tilt_mark, fill='#a3b0be')
        arc(0, tilt, tilt_mark, fill='#13799a', width=4)
        for point, angle in ((pan_mark, goal_pan), (tilt_mark, goal_tilt)):
            x, y, _ = project(point(angle))
            canvas.create_oval(x-4, y-4, x+4, y+4, outline='#153e52', fill='#ffffff', width=2)
        canvas.create_text(12, 12, anchor='nw', fill='#35546b', text='● 현재각   ○ 명령각')
