#!/usr/bin/env python3

import os.path as osp
import sqlite3
from flask import Flask, send_file, abort
from io import BytesIO

app = Flask(__name__)


@app.route("/tiles/<int:z>/<int:x>/<int:y>.png")
def serve_tile(z, x, y):
    try:
        conn = sqlite3.connect(osp.expanduser("~/.local/share/tobas/japan-latest.mbtiles"))  # TODO
        cursor = conn.cursor()
        # 画像データは、通常tilesテーブルのtile_data列に格納されている
        cursor.execute("SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?", (z, x, y))
        tile = cursor.fetchone()
        if tile:
            return send_file(BytesIO(tile[0]), mimetype="image/png")
        else:
            return abort(404)
    finally:
        conn.close()


if __name__ == "__main__":
    app.run(port=8080)
