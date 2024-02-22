# 例2 - ステートの実装ライフサイクル

`例2`のビヘイビアは、3つのステートを使って単純なステートマシンを構築します。

FlexBE システムを起動したら、FlexBE UI ダッシュボードから `Example 2` ビヘイビアをロードします。 
下の左端の画像はロード後の設定ダッシュボードを示しており、中央の画像は `ExampleState` プロパティが表示されたステートマシンを示しています。 
一番右の画像は、例の`A`ステートに入った後のステートマシンを示しています。

> 注：自律性が低い場合、自律性レベルによって遷移が待たされるため、
> 手動で`A`ステートに遷移するには、`Start`ステートの後に`done`遷移をクリックする必要があります。

<p float="center">
  <img src="../img/example2_dashboard.png" alt="読み込まれた例2。" width="30%">
  <img src="../img/example2_sm_property.png" alt="例2のステートマシンとプロパティ。" width="30%">
  <img src="../img/example2_a_state_enter.png" alt="実行中の例2のステートマシン。" width="30%">
</p>

`例1` の `LogState` に加えて、このビヘイビアでは、このリポジトリ `flexbe_turtlesim_demo_flexbe_states` の一部として提供されている `ExampleState` を使用します。 
`EventState`を継承した独自のFlexBEステート実装を開発するのは自由です。 ステートの実装をFlexBE に見つけさせるには、通常の ROS 2 にインストールされた Python スクリプトとして記述し、パッケージが `<flexbe_states />` を`export`するように指定してください。
```xml
<export>
    <build_type>ament_python</build_type>
    <flexbe_states />
</export>
```

この[`ExampleState`](../flexbe_turtlesim_demo_flexbe_states/flexbe_turtlesim_demo_flexbe_states/example_state.py)では、1つの引数(`target_time`)と2つの出力(`'done'`と`'failed'`)を指定しています。 
この例では `userdata` は使用しません。

```
「--」の後に引数を書いたものを一覧にします
-- target_time     float     ビヘイビアが始まってから経過している必要がある時間。

「<=」の後にラベル付けされた結果を書いたものを一覧にします（コンストラクタと一致しなければならない）
<= done            与えられた時間は過ぎた。
<= failed          失敗の結果の例。
```

これらは、必要なインスタンス変数とともに `__init__` メソッドで指定します。

```python
def __init__(self, target_time):
    """親クラスのコンストラクタに対応する引数を渡して、outcomes、input_keys、output_keysを宣言する。"""
    super().__init__(outcomes=['done', 'failed'])

    # 後で使用するためにステートの引数を保存する。
    self._target_wait_time = Duration(seconds=target_time)

    # コンストラクタは、実際にビヘイビアを開始するときではなく、ステートマシンを構築するときに呼び出される。
    # したがって、今は開始時刻を保存することはできず、後で保存することになる。
    self._state_start_time = None
    self._state_enter_time = None
    self._state_exit_time = None

    self._elapsed_time = Duration(nanoseconds=2**63 - 1)
```

このステートは、ステートに入った後、指定された時間待ってからdoneを返すのが仕事です。
このステートには2つの可能な結果が指定されていますが、このコードで実際に達成できるのは1つだけです。
データを保持するために、他のインスタンス属性を定義します。

ステートのライフサイクルは以下の通りです。
* `on_start` - ビヘイビアとすべてのサブステートがインスタンス化されるとき呼び出されます。
  * ビヘイビア全体と一緒に開始されるべきものを初期化するために、このメソッドを使います。 このメソッドはコンストラクタの後に呼び出されますが、`__init__`コンストラクタとは別のものです。

* `on_enter` - 上流からの遷移後にステートがアクティブになるとき呼び出されます。

* `execute` - Python の `None` 値が以外を返されるまで繰り返し呼び出されます。
  * `execute`メソッドの各「tic」は周期的なものです。必要であれば、ステートマシンレベルと個々のステートの両方で希望の更新tic頻度の値を指定できます。デフォルトの頻度は10Hzですが、全体的な希望値はステートマシン設定タブ（後述）で設定します。

  > 注：FlexBEでは、tic頻度は「ベストエフォート」であり、リアルタイムの性能保証はありません。

* `on_exit` は、ステートが `execute` から `None` 以外を返したときに一度だけ呼び出されます。

* `on_stop`はビヘイビアがシャットダウンされるときに呼び出されます。

`execute`メソッドだけは、再定義（オーバライド）して、ステートの処理を終了して値を返せるようにする必要があります。
他のメソッドは再定義するか、デフォルトの `EventState` の `pass` 値のままにしておくことができます。 
ある時点で、`execute` メソッドは `None` 以外の値を返すべきです。そうでなければ、ステートは永遠に実行されます。

[`ExampleState`](../flexbe_turtlesim_demo_flexbe_states/flexbe_turtlesim_demo_flexbe_states/example_state.py)は全てのメソッドを再定義し、各遷移にログ出力を追加して、ステートのライフサイクル中にシステムがどのように各メソッドを実行するかを示します。

```python
def on_enter(self, userdata):
    self._state_enter_time = ExampleState._node.get_clock().now()
    self._elapsed_time = Duration(seconds=0.0)
    self._return = None  # Clear return code on entry

    Logger.loginfo(f"on_enter for '{self._name}' state ({self.path}) @ {self.clock_time} "
                       f"- need to wait for {self.target_seconds} seconds.")

def on_exit(self, userdata):
    self._state_exit_time = ExampleState._node.get_clock().now()
    Logger.loginfo(f"on_exit for '{self._name}' state ({self.path}) @ {self.clock_time} "
                   f"elapsed time = {self.elapsed_seconds} seconds.")

def on_start(self):
    self._state_start_time = ExampleState._node.get_clock().now()
    Logger.loginfo(f"on_start for '{self._name}' state ({self.path}) @ {self.start_time} seconds "
                   f" time to wait = {self.target_seconds} seconds..")

def on_stop(self):
    self._elapsed_time = ExampleState._node.get_clock().now() - self._state_start_time
    Logger.loginfo(f"on_stop for '{self._name}' state ({self.path}) @ {self.clock_time} seconds "
                   f" total behavior instance elapsed time = {self.elapsed_seconds} seconds ")
    if self._state_enter_time is None:
        Logger.loginfo(f"on_stop for '{self._name}' state ({self.path}) @ {self.clock_time} seconds "
                   f" - never entered the state to execute! ")
    else:
        try:
            self._elapsed_time = self._state_exit_time - self._state_enter_time
            Logger.loginfo(f"    '{self._name}' state "
                        f"was active (enter-to-exit) for {self.elapsed_seconds} seconds.")
        except Exception as exc:  # pylint: disable=W0703
            Logger.logerr(f"  entered at time={self.enter_time} seconds but never exited!")
```

`Logger.loginfo`（および `logwarn`、`logerr`、`logdebug`）は `.ros/log` ファイル、オンボード端末、FlexBE UI にメッセージをログ出力します。

このステートの例では、データのログ出力を支援するために、以下のいくつかの補助プロパティメソッドも提供しています。

```python
@property
def elapsed_seconds(self):
    return f"{self._elapsed_time.nanoseconds/S_TO_NS:.3f}"

@property
def target_seconds(self):
    return f"{self._target_wait_time.nanoseconds/S_TO_NS:.3f}"

@property
def start_time(self):
    return f"{self._state_start_time.nanoseconds/S_TO_NS:.3f}"

@property
def enter_time(self):
    return f"{self._state_enter_time.nanoseconds/S_TO_NS:.3f}"

@property
def exit_time(self):
    return f"{self._state_exit_time.nanoseconds/S_TO_NS:.3f}"

@property
def clock_time(self):
    time_msg = self._node.get_clock().now().to_msg()
    time = time_msg.sec % 3600 + time_msg.nanosec/S_TO_NS
    return f"{time:.3f}"
```
ステートの実装では、必要に応じて追加の補助メソッドを自由に定義できます。

いくつかの重要なポイント
* 親クラス `EventState` はビヘイビアの ROS `node` への参照を保持しています。 ここでは `_node` 属性を使用して ROS クロックのインスタンスを取得しています。
* `for`ループ、`if-else`、`try-except`ブロックを含む通常の Python 構文は、いくつかの注意点を除いて、これらのメソッド内で有効です。
* `execute`だけが値を返すべきです。
* ステートは高速に動作する「反応が早い (reactive)」な状態でなければなりません。
    * プランニングのような長く実行される処理は、別のノードに譲って、できれば通常の `topics` と `actions` を使ったインターフェースにします。
* 繰り返しになりますが、これらのメソッドは長い「待たされる(blocking)」呼び出しであってはなりません。

> 注: 待たされる(blocking)呼び出しも可能ですが、`action` や 非同期サービスコースのような待たされない (non-blocking) 呼び出しを使用することをお勧めします。 
`action` と `service` の取り扱いについては、["Home"](home_behavior.md)、["Clear"](clear_behavior.md)、["Rotate"](rotate_behavior.md) のTurtleSim デモの議論を見てください。

[`ExampleState`](../flexbe_turtlesim_demo_flexbe_states/flexbe_turtlesim_demo_flexbe_states/example_state.py)の `execute` メソッドは `on_enter` からの時間を監視し、指定された更新頻度に基づいて *おおよその* 時間が経過したときに `done` を返します。
```python
def execute(self, userdata):
    """
    ステートがアクティブな間、このメソッドを周期的に実行する。

    主な目的は、ステートの条件をチェックし、対応する結果をトリガーすることである。
    結果が返されない場合、ステートはアクティブのままである。
    """
    if self._return is not None:
        # 自律性レベルによって待たされなければならない。
        # ここでは, 以前の結果を返すだけであり, 再計算は行わない.
        # ローカル情報はUIに送信されず、ログとターミナルにのみ表示される。
        Logger.localinfo(f"execute blocked for '{self._name}' state ({self.path}) @ {self.clock_time} "
                    f"- use prior return code={self._return}")
        return self._return

    # 通常の計算ブロック
    try:
        self._elapsed_time = ExampleState._node.get_clock().now() - self._state_enter_time
        if self._elapsed_time >= self._target_wait_time:
            Logger.loginfo(f"execute for '{self._name}' state ({self.path}) @ {self.clock_time} "
                           f"- done waiting at {self.elapsed_seconds} seconds.")
            self._return = 'done'
            return 'done'  # 上記で宣言した結果のひとつ
    except Exception:  # pylint:disable=W0703
        # 何かが間違っていた場合
        Logger.logerr(f"execute for '{self._name}' state ({self.path}) @ {self.clock_time} "
                       f"- something went wrong after {self.elapsed_seconds} seconds.")
        self._return = 'failed'
        return 'failed'

    # ローカル情報はUIには送信されず、ログとターミナルにのみ表示される。
    Logger.localinfo(f"execute for '{self._name}' state ({self.path}) @ {self.clock_time} "
                    f"- {self.elapsed_seconds} seconds since start.")
    return None  # これは、状態が実行を継続するための正常な動作である。
```

`execute`メソッドに関するいくつかの重要なポイントは以下の通りです。
* オペレータが、自律レベルのために終了遷移を待たされた場合に備えて、以前の `_return` 値を追跡します。 
遷移が待たされた場合、ステートは `execute` を呼び出し続けます。 
そのため、ステート設計者は特定の実装設計に応じて、実行をやり直すか、前の値を返すかを選択できます。 この柔軟性はステート開発者に任されています。 
しかし、遷移が待たされると `execute` が呼び出されることに注意してください。
これについては後ほどデモの例で説明します。
* この例では、コンソールへの大量のログ出力が含まれています。これは、特に実行ブロックでは、I/Oの計算コストのためにシステムの速度を低下させるので、非定型的なものです。
* このコードでは、例外処理と `failed` 結果の返り値の使い方の一例を示していますが、この例でこのブロックを使うことは想定していません。

さて、自律性「Low」で実行を開始する。

通常の実行中、上記の `Logger.localinfo` メソッド呼び出しは `.ros/log` ファイルとオンボード端末にログ出力するだけで、FlexBE UI にメッセージを送信しません。 
したがって、FlexBE コンソールには最終的な戻り値のみが表示されます。 
下の1番目の画像は、`execute` メソッドが最終的な戻り値まで呼び出され続け、その後 `on_exit` が呼び出されたときのオンボード端末の出力を示しています。
この結果に必要な自律性レベルは `off` であるため、システムはビヘイビアを終了し、すべてのステートに対して `on_stop` が呼び出されます。
下の2番目の画像は、ビヘイビアが完了し、システムの準備が整った後のFlexBE UIの最終出力を示しています。

<p float="center">
  <img src="../img/example2_onboard.png" alt="例2 オンボード端末の出力" height="30%">
  <img src="../img/example2_complete.png" alt="例2 FlexBE UIの最終出力" height="30%">
</p>

次の実行では、自律性レベルを「High」または「Full 」に高く設定してみてください。こうすると、ログステートの後にオペレータが「done」をクリックしなくても、ビヘイビアが完了するまで実行されます。「Off」に設定すると、オペレータがすべての遷移を確認する必要があります。

また、遷移ラベルの楕円をクリックして、早い遷移を強制的に実行してみてください。 ステートマシンを編集し、ダッシュボードの設定メッセージまたは待機時間を変更してみてください。 ビヘイビアを再保存する必要があります。

> 注：現状では、ビヘイビアは OCS マシンのワークスペースの `install` フォルダの下に保存されています。 
> これらの変更は、ソースフォルダには表示されず、パッケージが再ビルドれると失われます。
> 変更を保存するには、更新されたビヘイビア Python と xml マニフェストファイルをソースフォルダにコピーする必要があります。

`例2`で実験した後、[例3](../docs/example3.md)に進んで、ステートを「並列」に実行する`ConcurrencyContainter`を使った最初の階層型有限状態マシン(HFSM)を見てみましょう。

