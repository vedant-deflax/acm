# backend/main.py
"""
ACM (Autonomous Constellation Manager) — FastAPI application entry point.
Exposes all required APIs on port 8000.
"""

import logging
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from backend.api.routes import router

# ── Logging ───────────────────────────────────────────────────────────────── #
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("acm")


# ── App ───────────────────────────────────────────────────────────────────── #
@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info("ACM starting up — Autonomous Constellation Manager ready.")
    yield
    logger.info("ACM shutting down.")


app = FastAPI(
    title="Autonomous Constellation Manager",
    description="Orbital debris avoidance & constellation management system",
    version="1.0.0",
    lifespan=lifespan,
)

# Allow frontend (Vite dev server) to call the API
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ── Request timing middleware ─────────────────────────────────────────────── #
@app.middleware("http")
async def add_timing(request: Request, call_next):
    start = time.perf_counter()
    response = await call_next(request)
    elapsed_ms = (time.perf_counter() - start) * 1000
    response.headers["X-Process-Time-Ms"] = f"{elapsed_ms:.2f}"
    if elapsed_ms > 500:
        logger.warning(f"Slow request: {request.method} {request.url.path} took {elapsed_ms:.0f}ms")
    return response


# ── Global error handler ──────────────────────────────────────────────────── #
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    logger.error(f"Unhandled exception on {request.url.path}: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error", "error": str(exc)},
    )


# ── Register routes ───────────────────────────────────────────────────────── #
app.include_router(router)


@app.get("/")
async def root():
    return {
        "service": "Autonomous Constellation Manager",
        "version": "1.0.0",
        "docs": "/docs",
        "status": "/api/status",
    }


# ── Dev runner ────────────────────────────────────────────────────────────── #
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("backend.main:app", host="0.0.0.0", port=8000, reload=True)
