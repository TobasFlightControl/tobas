"""
外部のタイルサーバを QML の OSM プラグインの形式 (.../{z}/{x}/{y}.png) で提供するウェブサーバ．
"""

from __future__ import annotations

from os import environ as env

import httpx
import uvicorn
from fastapi import FastAPI, HTTPException, Response

UPSTREAM_TEMPLATE = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}.png"

app = FastAPI()
_client: httpx.AsyncClient | None = None


@app.on_event("startup")
async def _startup():
    global _client

    timeout = float(env.get("TILE_PROXY_TIMEOUT", "5.0"))

    _client = httpx.AsyncClient(
        headers={"User-Agent": "Tobas (QtLocation tile proxy)"},  # User-Agent が空だとレート制限が入る場合がある
        timeout=timeout,
        follow_redirects=True,
    )


@app.on_event("shutdown")
async def _shutdown():
    global _client

    if _client is not None:
        await _client.aclose()
        _client = None


@app.get("/healthz")
async def healthz():
    return Response(content=b"ok\n", media_type="text/plain; charset=utf-8")


@app.get("/tiles/{z:int}/{x:int}/{y:int}.png")
async def tiles(z: int, x: int, y: int):
    if _client is None:
        raise HTTPException(status_code=500, detail="HTTP client not initialized")

    upstream_url = UPSTREAM_TEMPLATE.format(z=z, x=x, y=y)
    r = await _client.get(upstream_url)

    if r.status_code != 200:
        raise HTTPException(status_code=r.status_code, detail=f"Upstream error ({r.status_code})")

    return Response(content=r.content, media_type="image/png")


def main(args=None) -> None:
    host = env.get("TILE_PROXY_HOST", "127.0.0.1")
    port = int(env.get("TILE_PROXY_PORT", "8080"))
    uvicorn.run(app, host=host, port=port, reload=False, log_level="warning")


if __name__ == "__main__":
    main()
