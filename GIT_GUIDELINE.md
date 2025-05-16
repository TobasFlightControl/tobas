# Git Guideline

## Main Branches

- `${ros-distribution}`: 各 ROS ディストリビューションにおいて動作が保証された最新のブランチ．リリースはここから作る．
- `${ros-distribution}-develop`: `${ros-distribution}`に対する開発用ブランチ．動作は保証されない．

## Feature Branches

`${ros-distribution}-develop`から生やす．

### Branch Namespaces

- `feat/`: For new features
- `fix/`: For bug fixes
- `remove/`: For file removals
- `docs/`: For documentation updates
- `style/`: For style changes (formatting, spacing, etc.)
- `refactor/`: For code refactoring (no functional changes)
- `perf/`: For performance improvements
- `test/`: For adding or fixing tests
- `build/`: For build system changes
- `ci/`: For CI/CD configuration changes
- `change/`: For small changes or tweaks
- `chore/`: For maintenance tasks

### Example Branch Names

- `feat/user-registration`
- `fix/product-price-validation`
- `docs/readme-update`
- `style/button-styling`

## Commit Message

### Format

```txt
<Type>: <Subject>

<Body>
```

#### Type

- `feat`: A new feature
- `fix`: A bug fix
- `docs`: Documentation changes
- `style`: Changes that do not affect code behavior (e.g., formatting)
- `refactor`: Code changes without affecting functionality
- `perf`: Performance improvements
- `test`: Adding or fixing tests
- `build`: Changes to the build system or external dependencies
- `ci`: Changes to CI/CD scripts
- `chore`: Maintenance tasks or other minor changes
- `wip`: Work in progress

#### Subject

- コミットの概要
- 命令形
- 50 字以内

#### Body (Optional)

- コミットの詳細
- 1 行あたり 72 字以内

### Example Commit Messages

- `feat: add user registration support`
- `fix: resolve price validation bug`
- `docs: update API usage instructions`
- `style: adjust button alignment`
- `perf: optimize database queries`

## 参考

- [backend-SafeTrust/GIT_GUIDELINE.md | GitHub](https://github.com/safetrustcr/backend-SafeTrust/blob/main/GIT_GUIDELINE.md)
- [【Git】コミットに規約をつくる | Qiita](https://qiita.com/Kenya/items/f72fba8fecc79d1b090c)
