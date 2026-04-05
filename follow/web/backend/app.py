#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import os
from contextlib import asynccontextmanager

import numpy as np
import cv2
from av import VideoFrame
from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
import rclpy

from ros_bridge import SharedState, start_ros_node


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(BASE_DIR)
FRONTEND_DIR = os.path.join(PROJECT_DIR, "frontend")

shared_state = SharedState()
ros_node, ros_thread = start_ros_node(shared_state)
pcs = set()


@asynccontextmanager
async def lifespan(app: FastAPI):
    yield

    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros, return_exceptions=True)
    pcs.clear()

    try:
        if ros_node is not None:
            ros_node.destroy_node()
    except Exception:
        pass

    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


app = FastAPI(lifespan=lifespan)


class CameraVideoTrack(VideoStreamTrack):
    def __init__(self, shared_state: SharedState):
        super().__init__()
        self.shared_state = shared_state

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        with self.shared_state.lock:
            frame = self.shared_state.latest_frame.copy() if self.shared_state.latest_frame is not None else None

        if frame is None:
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "Waiting for /image_proc ...",
                (60, 360),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (255, 255, 255),
                2,
            )

        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


@app.get("/")
async def index():
    return FileResponse(os.path.join(FRONTEND_DIR, "index.html"))


@app.post("/offer")
async def offer(request: Request):
    params = await request.json()

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print("Connection state:", pc.connectionState)
        if pc.connectionState in ["failed", "closed", "disconnected"]:
            await pc.close()
            pcs.discard(pc)

    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    await pc.setRemoteDescription(offer)

    track = CameraVideoTrack(shared_state)
    pc.addTrack(track)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return JSONResponse({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type,
    })

@app.post("/select_bbox_click")
async def select_bbox_click(request: Request):
    data = await request.json()

    click_x = float(data["click_x"])
    click_y = float(data["click_y"])

    disp_left = float(data["disp_left"])
    disp_top = float(data["disp_top"])
    disp_width = float(data["disp_width"])
    disp_height = float(data["disp_height"])

    with shared_state.lock:
        frame_w = float(shared_state.frame_width)
        frame_h = float(shared_state.frame_height)

    if frame_w <= 0 or frame_h <= 0 or disp_width <= 1 or disp_height <= 1:
        return JSONResponse(
            {"ok": False, "error": "invalid frame/display size"},
            status_code=400
        )

    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    # screen -> image
    cx = (click_x - disp_left) * frame_w / disp_width
    cy = (click_y - disp_top) * frame_h / disp_height

    cx = clamp(cx, 0.0, frame_w - 1.0)
    cy = clamp(cy, 0.0, frame_h - 1.0)

    # kích thước bbox cố định
    w = 260.0
    h = 380.0

    x = cx - w / 2.0
    y = cy - h / 2.0

    x = clamp(x, 0.0, max(0.0, frame_w - w))
    y = clamp(y, 0.0, max(0.0, frame_h - h))

    ros_node.publish_select_bbox(x, y, w, h)

    return {
        "ok": True,
        "bbox": [x, y, w, h]
    }


@app.post("/click")
async def click(request: Request):
    data = await request.json()
    x = float(data["x"])
    y = float(data["y"])
    ros_node.publish_click(x, y)
    return {"ok": True}


@app.post("/reset")
async def reset(request: Request):
    ros_node.publish_reset()
    return {"ok": True}


@app.get("/ai_state")
async def ai_state():
    with shared_state.lock:
        return {
            "frame_width": shared_state.frame_width,
            "frame_height": shared_state.frame_height,
        }


app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")