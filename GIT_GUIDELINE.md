# Git Guideline

## Branches

- `${ros-distribution}`: 各 ROS ディストリビューションにおいて動作が保証された最新のブランチ．
- `${ros-distribution}-develop`: `${ros-distribution}`に対する開発用ブランチ．動作は保証されない．
- `feature/`: 機能実装用のブランチ．`${ros-distribution}-develop`から派生する．マージ後に削除する．
- `release/`: 後方互換性が保証された各リリース用のブランチ．`${ros-distribution}`から派生する．ここからリリース`vx.x.x`を作る．
- `hotfix/`: 緊急バグ対応用ブランチ．`${ros-distribution}`またはリリースブランチから派生し，マージ後に削除する．

### Example Branch Names

- `jazzy`
- `jazzy-develop`
- `feature/nonlinear-mpc`
- `release/v2.5`
- `hotfix/mag-drivers`

## Commit Message

### Format

```txt
<Type>: <Subject>

<Body>
```

#### Type

- `add`: 機能，ファイル追加
- `fix`: バグ修正
- `modify`: 軽微な変更，調整
- `change`: 仕様変更
- `remove`: 機能，ファイル削除

#### Subject

- コミットの概要
- 50 字以内
- 文頭大文字
- ピリオドなし
- 命令形

#### Body (Optional)

- コミットの詳細
- 1 行あたり 72 字以内
- How ではなく What と Why

### Example Commit Messages

- `fix: Resolve price validation bug`

## 参考

- [Git での基本的な開発フローについて](https://qiita.com/jun1s/items/e45761f103c52926d5e5)
- [【Git】コミットに規約をつくる | Qiita](https://qiita.com/Kenya/items/f72fba8fecc79d1b090c)
