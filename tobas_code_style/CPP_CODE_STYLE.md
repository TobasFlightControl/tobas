# C++ Code Style

## Example

See [tobas_cpp_code_style_example](./tobas_cpp_code_style_example/).

## C++ バージョン

- C++23
- 非標準拡張は使用しない

```cmake
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```

## g++

- g++-13
- コンパイルオプション: `-Wall -Wextra -Wpedantic -Wshadow -Werror`
- PIC を有効化

```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
```

## 拡張子

- `.hpp`, `.cpp`のみ．

## ヘッダファイル

- 以下の例外を除く全ての`.cpp`ファイルは同名の`.hpp`ファイルをもつ．つまり，複数のヘッダファイルの実装を 1 箇所にまとめてはならない．
  - main 関数をもつ
  - プラグイン: ROS 2 コンポーネント，Gazebo プラグインなど
- インクルードガード: `#pragma once`
- 使用されているシンボルが定義されているヘッダファイルを直接インクルードする．再帰的なインクルードには極力依存しない．
- 前方宣言は極力使用しない．
- 関数の宣言と定義を分ける．
- 定義が短く且つ速度が重視される場合はインライン指定する．
- インクルード順
  1. Related header file
  1. A blank line
  1. C system headers, and any other headers in angle brackets with the .h extension, e.g., <unistd.h>, <stdlib.h>.
  1. A blank line
  1. C++ standard library headers (without file extension), e.g., <algorithm>, <cstddef>.
  1. A blank line
  1. Third-party libraries' header files.
  1. A blank line
  1. Tobas libraries' header files
  1. A blank line
  1. Your project's header files.
- それぞれのセクションのヘッダファイルをアルファベット順に並べる．
- 関連ヘッダのインクルードに UNIX ディレクトリエイリアスは使用しない: `../include/my_library/my_class.hpp` -> `my_library/my_class.hpp`
- 同じライブラリのヘッダのパスのみダブルクオーテーションで囲む．

## 名前空間

- 全てのシンボルを`tobas`名前空間に含める．
- 別の cpp ファイルからインクルードされ得るシンボルは`tobas`以下の適切な名前空間に含める．
- `using namespace`: ヘッダファイルでは不可．ソースファイルでも極力使用しない．
- cpp ファイル内でシンボルを定義し，内部リンケージにのみ使用する場合は，無名名前空間に入れて他の翻訳単位からは参照されないことを明示する．
- 関数や変数のスコープはできる限り狭く，できる限り使用直前に定義する．
- グローバル変数は`constexpr`でのみ定義可能．

## クラス

- コンストラクタではエラーハンドリングが難しいため，失敗する可能性のあるコードをコンストラクタに書かない．代わりに`initialize()`メソッドを定義する．
- 仮想メソッドのオーバーライドされた実装は基底クラスのコンストラクタ時点では利用できないので，基底クラスのコンストラクタで仮想メソッドを呼ばない．
- 暗黙的な型変換を防ぐため，コンストラクタにはなるべく`explicit`をつける．
- コピーコンストラクタ，ムーブコンストラクタを定義し，それぞれが可能かどうかを明示する．
- `public`なデータのみを含む場合は`struct`を使う．それ以外は基本的に`class`を使う．
- 意味を明確にするために`std::pair`や`std::tuple`ではなく`struct`を使う．
- 継承には`public`をつける．
- 演算子オーバーロードは意味が明確なときのみ可能．
- メンバ変数は基本`private`で必要最小限のアクセッサを実装するのが望ましいが，正当な理由をコメントすれば他のアクセス権も認める．
- アクセスグループの定義順
  1. public
  1. protected
  1. private
- アクセスグループ内の定義順
  1. Types and type aliases (typedef, using, enum, nested structs and classes, and friend types)
  1. (Optionally, for structs only) non-static data members
  1. Static constants
  1. Factory functions
  1. Constructors and assignment operators
  1. Destructor
  1. All other functions (static and non-static member functions, and friend functions)
  1. All other data members (static and non-static)
- `friend`はなるべく使わず，`public`メンバでのみ外とやりとりする．

## 関数

- 可読性とパフォーマンスの面から，出力は引数よりも返り値の方が望ましい．
- 入力引数: プリミティブ型ならコピー，非プリミティブ型なら const 参照．
- 出力引数: 非 const 参照．
- 入力引数，出力引数の順に並べる．
- 関数の役割は最小限に．1 つの関数は 1 つのことを．
- オーバーロードは関数の意味が変わらず同じ docstrings で説明できる場合のみ．
- 未定義動作を起こす恐れがあるため，仮想関数の引数にデフォルト値を与えてはならない．
- 返り値は`auto`ではなく明確に定義する．

## ポインタ

- なるべくポインタではなく参照を使う．
- ポインタを使うにしても，生ポインタではなくスマートポインタを使う．
- 所有権を明確にするため，`std::shared_ptr`よりも`std::unique_ptr`が望ましい．
- `std::auto_ptr`は使用不可．
- `NULL`ではなく`nullptr`を使う．

## 例外

- あらゆる関数は例外を吐いてはならない．
- 返り値を`bool`かエラーコードにしてハンドリングする．
- 適切にエラーハンドリングした上で`noexcept`を指定する．

## キャスト

- ダウンキャストは極力しない．
- C スタイルキャストではなく C++スタイルを使う: `(int)x` -> `static_cast<int>(x)`

## インクリメント

- 特に理由がない限りは前置インクリメントを使用: `i++` -> `++i`

## const

- 指定可能な全ての場所に`constexpr`をつける．
- 指定可能な全ての場所に`const`をつける．

## 整数型

- `int`以外のビルトイン整数型は使用しない．
- なるべく用途に合った`size_t`，`strdiff_t`，`time_t`などを使う．
- その他整数のサイズを保証する場合は`stdint.h`に定義されている整数型を使う．
- 符号なし整数型はビットパターンなど特殊な場合を除き使用せず，符号は`assert`で保証する．

## 不動小数型

- `float`または`double`のみ使用可．
- `long double`などは使用不可．

## 命名規則

- 長くなってもよいので読む人が容易に意味や目的を理解できるような名前を付ける．

### ケース

- ファイル: `hoge_fuga.hpp`, `hoge_fuga.cpp`
- 型 (class, struct, type alias, enum, type template parameter): `HogeFuga`
- ローカル変数: `hoge_fuga`
- グローバル変数 (非推奨): `g_hoge_fuga`
- public 変数: `hoge_fuga`
- protected, private 変数: `hoge_fuga_`
- 引数: `_hoge_fuga`
- constexpr 定数: `kHogeFuga`
- 関数: `hogeFuga()`
- 名前空間: `hoge_fuga`
- 列挙子 (enum の要素): `kHogeFuga`
- マクロ (非推奨): `HOGE_FUGA`

## コメント

- doc コメント (クラスコメント，関数コメント): `/** @brief ... */`
- 変数コメント: `/* ... */` or `/** @brief ... */`
- 実装コメント: `// ...`
- 一時的なコメントアウト: `// ...`
- ライセンスボイラープレートやヘッダ上部の説明文は不要 <!-- TODO: Add file comments: https://google.github.io/styleguide/cppguide.html#File_Comments -->
- コメントは日本語でも構わない <!-- TODO: English only -->

## フォーマット

- [.clang-format](./.clang-format) に従う．

## ROS 2

- 不特定多数へのゼロコピー転送のため，トピックは`UniquePtr`で発行して`ConstSharedPtr`で購読する．

## 参考

- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [Code styleand language versions | ROS 2 Documentation](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#id1)
