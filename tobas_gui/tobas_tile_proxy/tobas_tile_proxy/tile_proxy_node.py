"""
外部のタイルサーバを QML の OSM プラグインの形式 (.../{z}/{x}/{y}.png) で提供するウェブサーバ．
"""

from __future__ import annotations

from argparse import ArgumentParser

import httpx
import uvicorn
from fastapi import FastAPI, HTTPException, Response


g_app = FastAPI()
g_client: httpx.AsyncClient | None = None

g_upstream: str
g_timeout: float


@g_app.on_event("startup")
async def _startup():
    global g_client

    g_client = httpx.AsyncClient(
        headers={"User-Agent": "Tobas (QtLocation tile proxy)"},  # User-Agent が空だとレート制限が入る場合がある
        timeout=g_timeout,
        follow_redirects=True,
    )


@g_app.on_event("shutdown")
async def _shutdown():
    global g_client

    if g_client is not None:
        await g_client.aclose()
        g_client = None


@g_app.get("/healthz")
async def healthz():
    return Response(content=b"ok\n", media_type="text/plain; charset=utf-8")


@g_app.get("/tiles/{z:int}/{x:int}/{y:int}.png")
async def tiles(z: int, x: int, y: int):
    if g_client is None:
        raise HTTPException(status_code=500, detail="HTTP client not initialized")

    upstream_url = g_upstream.format(z=z, x=x, y=y)
    r = await g_client.get(upstream_url)

    if r.status_code != 200:
        raise HTTPException(status_code=r.status_code, detail=f"Upstream error ({r.status_code})")

    return Response(content=r.content, media_type="image/png")


def main(args=None) -> None:
    global g_upstream
    global g_timeout

    # Get arguments
    parser = ArgumentParser()
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--upstream",
        type=str,
        default="https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    args, _ = parser.parse_known_args(args)

    # Set global parameters
    g_upstream = args.upstream
    g_timeout = args.timeout

    # Run proxy
    uvicorn.run(g_app, host=args.host, port=args.port, reload=False, log_level="warning")


if __name__ == "__main__":
    main()
