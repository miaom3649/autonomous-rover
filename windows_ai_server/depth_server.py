"""Windows GPU metric-depth inference server."""

import argparse
import io
import time
from contextlib import asynccontextmanager

import numpy as np
import torch
from fastapi import FastAPI, Request, Response
from PIL import Image
from transformers import AutoImageProcessor, AutoModelForDepthEstimation


MODEL_ID = "Intel/zoedepth-nyu"
_model = None
_processor = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _model, _processor
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Loading {MODEL_ID} on {device}...")
    _processor = AutoImageProcessor.from_pretrained(MODEL_ID)
    _model = AutoModelForDepthEstimation.from_pretrained(MODEL_ID).to(device)
    _model.eval()
    print("Model ready — server accepting requests.\n")
    yield


app = FastAPI(lifespan=lifespan)


@app.post("/depth")
async def infer_depth(request: Request) -> Response:
    image = Image.open(io.BytesIO(await request.body())).convert("RGB")
    started = time.perf_counter()
    inputs = _processor(images=image, return_tensors="pt")
    device = next(_model.parameters()).device
    inputs = {name: value.to(device) for name, value in inputs.items()}
    with torch.no_grad():
        predicted = _model(**inputs).predicted_depth
    while predicted.dim() < 4:
        predicted = predicted.unsqueeze(0)
    width, height = image.size
    resized = torch.nn.functional.interpolate(
        predicted, size=(height, width), mode="bicubic", align_corners=False
    )
    depth = resized.squeeze().cpu().numpy().astype(np.float32)
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    print(f"depth {width}x{height}: {elapsed_ms:.0f}ms")
    return Response(
        content=depth.tobytes(),
        media_type="application/octet-stream",
        headers={"X-Depth-Height": str(height), "X-Depth-Width": str(width)},
    )


@app.get("/health")
def health() -> dict:
    return {"status": "ok", "model": MODEL_ID, "cuda": torch.cuda.is_available()}


if __name__ == "__main__":
    import uvicorn

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args()
    uvicorn.run(app, host=args.host, port=args.port)
