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
from transformers import AutoModelForDepthEstimation, AutoImageProcessor

MODEL_ID = "Intel/zoedepth-nyu"
_model = None
_processor = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _model, _processor
    device = "cuda" if torch.cuda.is_available() else "cpu"
    device_label = "GPU (CUDA)" if device == "cuda" else "CPU"
    print(f"Loading {MODEL_ID} on {device_label}...")
    _processor = AutoImageProcessor.from_pretrained(MODEL_ID)
    _model = AutoModelForDepthEstimation.from_pretrained(MODEL_ID).to(device)
    _model.eval()
    print("Model ready — server accepting requests.\n")
    yield


app = FastAPI(lifespan=lifespan)


@app.post("/depth")
async def infer_depth(request: Request) -> Response:
    body = await request.body()
    pil_img = Image.open(io.BytesIO(body)).convert("RGB")

    t0 = time.perf_counter()
    inputs = _processor(images=pil_img, return_tensors="pt")
    device = next(_model.parameters()).device
    inputs = {k: v.to(device) for k, v in inputs.items()}
    with torch.no_grad():
        outputs = _model(**inputs)

    # outputs.predicted_depth: metric depth in meters at model native resolution
    predicted = outputs.predicted_depth
    while predicted.dim() < 4:
        predicted = predicted.unsqueeze(0)
    w, h = pil_img.size
    resized = torch.nn.functional.interpolate(
        predicted, size=(h, w), mode="bicubic", align_corners=False
    )
    depth_f32 = resized.squeeze().cpu().numpy().astype(np.float32)
    dt_ms = (time.perf_counter() - t0) * 1000

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
