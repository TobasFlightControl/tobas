import os.path as osp
import sqlite3
from flask import Flask, send_file, abort
from io import BytesIO

app = Flask(__name__)


@app.route("/tiles/<int:z>/<int:x>/<int:y>.png")
def serve_tile(z: int, x: int, y: int):
    try:
        conn = sqlite3.connect(osp.expanduser("~/.local/share/tobas/output.mbtiles"))  # TODO
        cursor = conn.cursor()

        # クエリで圧縮画像を取得
        cursor.execute(
            "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
            (z, x, y),
        )
        data = cursor.fetchone()
        if not data:
            return abort(404)  # Not Found
        image: bytes = data[0]

        # TODO: dataの先頭からpngであることを確認
        header = image[:8]
        if header != b"\x89PNG\r\n\x1a\n":
            print("The byte sequence obtained from the MBTiles file is not in PNG format.")
            return abort(505)

        return send_file(BytesIO(image), mimetype="image/png")

    finally:
        conn.close()


if __name__ == "__main__":
    app.run(port=8080)
