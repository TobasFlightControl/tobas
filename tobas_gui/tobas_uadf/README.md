# Universal Aircraft Description Format

## 仕様

XACRO の航空機拡張．
航空機の運動方程式 (力を求めるまで) に関する部分を記述する．

### 特殊なジョイント

- `thrust`: 推進モジュール
  - `direction`: 回転方向
    - `value`: `cw`/`ccw`
- `cs`: 固定翼の操舵面
- `tilt`: ティルトジョイント
