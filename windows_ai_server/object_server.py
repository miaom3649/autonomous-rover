"""Windows GPU object-detection server using a small YOLO model."""

import argparse
import io
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI, Request
from PIL import Image
from ultralytics import YOLO


MODEL_ID = "yolo11n.pt"
_model = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _model
    print(f"Loading {MODEL_ID}...")
    _model = YOLO(MODEL_ID)
    print("YOLO ready — server accepting requests.\n")
    yield


app = FastAPI(lifespan=lifespan)


@app.post("/detect")
async def detect(request: Request) -> dict:
    image = Image.open(io.BytesIO(await request.body())).convert("RGB")
    width, height = image.size
    started = time.perf_counter()
    result = _model.predict(image, imgsz=640, conf=0.4, verbose=False)[0]
    detections = []
    for box in result.boxes:
        x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
        class_id = int(box.cls[0])
        detections.append(
            {
                "label": result.names[class_id],
                "confidence": float(box.conf[0]),
                "x1": x1 / width,
                "y1": y1 / height,
                "x2": x2 / width,
                "y2": y2 / height,
            }
        )
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    print(f"detect {width}x{height}: {len(detections)} objects, {elapsed_ms:.0f}ms")
    return {"detections": detections, "inference_ms": elapsed_ms}


@app.get("/health")
def health() -> dict:
    return {"status": "ok", "model": MODEL_ID}


if __name__ == "__main__":
    import uvicorn

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args()
    uvicorn.run(app, host=args.host, port=args.port)
