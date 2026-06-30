"""
Windows depth inference server.
Run on Windows host (not in VM) to use the NVIDIA GPU.

Usage:
    python depth_server.py [--port 8765]

Returns float32 depth maps in meters for each POSTed JPEG image.
"""
import argparse
import io
import time
from contextlib import asynccontextmanager

import numpy as np
import torch
from fastapi import FastAPI, Request, Response
from PIL import Image
from transformers import pipeline

MODEL_ID = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"
_pipe = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _pipe
    device = 0 if torch.cuda.is_available() else -1
    device_label = f"GPU (CUDA:{device})" if device >= 0 else "CPU"
    print(f"Loading {MODEL_ID} on {device_label}...")
    _pipe = pipeline(
        task="depth-estimation",
        model=MODEL_ID,
        device=device,
    )
    print("Model ready — server accepting requests.\n")
    yield


app = FastAPI(lifespan=lifespan)


@app.post("/depth")
async def infer_depth(request: Request) -> Response:
    body = await request.body()
    pil_img = Image.open(io.BytesIO(body)).convert("RGB")

    t0 = time.perf_counter()
    result = _pipe(pil_img)
    dt_ms = (time.perf_counter() - t0) * 1000

    depth_map: np.ndarray = result["depth"]
    depth_f32 = np.asarray(depth_map, dtype=np.float32)

    h, w = depth_f32.shape
    print(f"depth {w}x{h}  min={depth_f32.min():.2f}m  max={depth_f32.max():.2f}m  {dt_ms:.0f}ms")

    return Response(
        content=depth_f32.tobytes(),
        media_type="application/octet-stream",
        headers={"X-Depth-Height": str(h), "X-Depth-Width": str(w)},
    )


@app.get("/health")
def health():
    return {"status": "ok", "model": MODEL_ID, "cuda": torch.cuda.is_available()}


if __name__ == "__main__":
    import uvicorn

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args()

    uvicorn.run(app, host=args.host, port=args.port)
