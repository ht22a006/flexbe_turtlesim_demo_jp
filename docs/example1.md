# 例1 - 基本的なFlexBEのステートの実装

最初のビヘイビアである`例1`は、FlexBEビヘイビアエンジンで提供される2つのステートを使ってシンプルなステートマシンを構築します。

以下のどちらかの方法でFlexBEを開始します。まとめて

`ros2 launch flexbe_app flexbe_full.launch.py use_sim_time:=False`

または、個々のコンポーネントで

  `ros2 launch flexbe_onboard behavior_onboard.launch.py use_sim_time:=False`

  `ros2 run flexbe_mirror behavior_mirror_sm --ros-args --remap __node:="behavior_mirror" -p use_sim_time:=False`

  `ros2 run flexbe_app run_app --ros-args --remap name:="flexbe_app" -p use_sim_time:=False`

  `ros2 run flexbe_widget be_launcher --ros-args --remap name:="behavior_launcher" -p use_sim_time:=False`

下の左端の画像のように、FlexBE UIダッシュボードから`Example 1`のビヘイビアを読み込みます。
読み込むと、下図中央に示すように「Behavior Dashboard」にビヘイビア設定情報が表示されます。
このビヘイビアでは、オペレータが設定可能な「パラメータ」 `waiting_time` と、定数のプライベート設定変数 `log_msg` が定義されています。
左上の 「Overview」の領域には、ビヘイビア名「Example 1」、説明、作者情報が表示されます。 
ビヘイビア名は、実装のPythonファイル名 `example_1_sm.py` とクラス名 `Example1SM` に変換されます。
Pythonファイルに加えて、ビヘイビアマニフェスト `example_1.xml` も作成されます。

<p float="center">
  <img src="../img/example1_loading.png" alt="例1の読み込み。" width="30%">
  <img src="../img/example1_config.png" alt="例1の設定。" width="30%">
  <img src="../img/example1_sm.png" alt="例1のステートマシーン。" width="30%">
</p>

「例1」ビヘイビアのFlexBE Behavior editor ビュー。 画像をクリックすると、高解像度の注釈付き画像が表示されます。

「Statemachine Editor」のタブでは、上の一番右の画像のように、ロードした既存のステートマシンを見ることができます。
ステートマシンを編集して保存することもできますが、今は探索だけします。 ステートをダブルクリックすると、ステート プロパティ編集ボックスが開きます。
例1では、以下に示す FlexBE の標準的な [`LogState`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_states/flexbe_states/log_state.py) と [`WaitState`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_states/flexbe_states/wait_state.py) の実装を使用しています。

記録 (log) ステートは最も単純なFlexBEステートの実装の一つです。
```python
from flexbe_core import EventState, Logger


class LogState(EventState):
    """
    定義済みのメッセージを記録できるステート。

    ビヘイビアに何が起こったかをオペレーターに正確に伝えるために使用できる。

    -- text      string  端末にログされるメッセージ。
    -- severity  uint8   ログの種類 (Logger.REPORT_INFO / WARN / HINT / ERROR)

    <= done     メッセージが記録されたことを示す。
    """

    def __init__(self, text, severity=Logger.REPORT_HINT):
        super(LogState, self).__init__(outcomes=['done'])
        self._text = text
        self._severity = severity

    def execute(self, userdata):
        # すでに記録されている。何も待つ必要はない。
        return 'done'

    def on_enter(self, userdata):
        """ステートに入る際に記録する"""
        Logger.log(self._text, self._severity)
```

ステートの実装は、ステートの実際の実行を提供するPythonスクリプトです。
この例では、FlexBE Logger クラスをインポートしています。このクラスは、オンボード端末、標準 FlexBE ログファイル、およびオペレータの FlexBE UI にメッセージを表示するために使用されます。 
`__init__` の引数 `text` と `severity` は、上の一番右の画像に示すように、ステートプロパティのエディタボックスに表示されます。
可能な結果 `done` も同様に表示され、エディタでは自律遷移に必要な自律性レベルを設定することができます。

ステートの実装は、FlexBE が `flexbe_core` パッケージで提供する [`EventState`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_core/flexbe_core/core/event_state.py) クラスを継承して定義しなければなりません。

`"""`マーク内のPython doc-stringは、ステートに関する情報を提供し、UIがデータの取り扱いに関する情報を表示するために使用されます。 特別なインジケータは異なるデータに対して使用されます。

* `--` は `__init__` で定義された引数を表します。
  * この場合、 `text` と `severity` はインスタンス時に指定されます。
* `<=` はステートの可能な結果を表します。
  * この場合、`done`だけが可能な結果です。
  * この情報は `EventState`の`super().__init__` メソッド呼び出しで指定された `outcomes=['done']` と一致しなければなりません。

この例では示していませんが、その他のUI仕様には以下のものがあります。
* `>#` - 上流のステートから入力 `userdata` として渡されるデータ 
* `#>` - 下流のステートへ出力 `userdata` として渡されるデータ

> 注：userdataは、ステートマシンのレベルでも定義できます。

`LogState` には `on_enter` メソッドと `execute` メソッドがあります。
`on_enter` メソッドは、遷移によってステートに最初に入ったときに呼び出されます。 この場合、ステートは`Logger` クラスを使用して、端末、ログファイル、FlexBE UI にデータを記録 (log) します。

`execute`メソッドは `None` 以外の値を返すまで、指定された頻度で呼び出されます。
返される値は、指定された有効な値（例えば、この場合は `done`）であることが期待されます（そして、実行時に強制されます）。
`LogState`の場合、`execute`メソッドは何もすることがないので、すぐに`done`を返します。

2番目のステートは、以下に示す[`WaitState`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_states/flexbe_states/wait_state.py)ステートの実装です。

```python
from flexbe_core import EventState


class WaitState(EventState):
    """
    時間制限のあるプロセスを待つために使用できるステートの実装。

    -- wait_time  float  待機時間（秒）。

    <= done       待ち時間が経過したことを示す。
    """

    def __init__(self, wait_time):
        super(WaitState, self).__init__(outcomes=['done'])
        self._wait = wait_time

    def execute(self, userdata):
        elapsed = WaitState._node.get_clock().now() - self._start_time
        if elapsed.nanoseconds * 10 ** -9 > self._wait:
            return 'done'

        return None

    def on_enter(self, userdata):
        """このステートに入ると、現在時刻を保存して待機を開始する。"""
        self._start_time = WaitState._node.get_clock().now()
```

`wait_time`パラメータは、ステートに入ってから戻るまでの待ち時間を指定します。
これはステートマシンの中で単純な遅延を与えます。
このステートも結果は `done` だけです。
上の一番右の図に示すように、 `WaitState` 実装の `Wait_after_logging` という名前のステートインスタンスでは、`wait_time` パラメータにオペレータが設定可能なパラメータ `self.waiting_time` を設定しています。

これらのビューを探索した後、FlexBE UIの 「Runtime Control」タブに移動します。

下の一番左の画像は、ビヘイビアを実行する前の初期表示です。 オペレータはここで`wait_time`パラメーター値（現在は3秒）を調整し、初期の監視自律レベルを設定することができます。 
この場合、「Off」よりも高い値を必要とする遷移はブロックします。

> 注：このビヘイビアのために、`wait_time`は初期状態では小数点なしの`3`として設定されています。 
> これは入力として整数値を必要とします。 浮動小数点値を許可するには、小数点付きで指定してください（例：`3.`）。

中央の画像は初期状態の `Print_Message` で、必要な自律性レベルのために出力がブロックされています。 
システムは、次のステートへの遷移を有効にするために、楕円形のラベルの「done」遷移をクリックするようオペレータに要求します。
「Behavior Feedback」の領域には、設定画面で元々定義されていた「Hello World！」メッセージを含む、オンボードビヘイビアから記録された出力が表示されます。 このメッセージは、オンボードノードが起動したオンボード端末ウィンドウにも記録されます。
「done」遷移をクリックした後、現在のアクティブなステートは、下の一番右の画像に示すように、`Wait_After_Logging`ステートに遷移します。
これは待機期間中なので、出力遷移は灰色で表示され、中央の画像はアクティブな遷移が黄色で強調表示されています。
オペレータはステートが終了するのを待つか、待機時間が終了する前にラベルの楕円をクリックしてステートを先取りし、「done」遷移を強制することを選択できます。 
このステートは自律性レベルが「off」であるため、ビヘイビアは自律的に完了し、一番左の画像に示されている「Start」領域に戻ると、結果が「finished」に戻ります。

<p float="center">
  <img src="../img/example1_start_low.png" alt="例1 開始画面" width="30%">
  <img src="../img/example1_low_block.png" alt="例1 ログメッセージ" width="30%">
  <img src="../img/example1_wait.png" alt="例1 待機中." width="30%">
</p>

次の実行では、自律性レベルをより高く「High」または 「Full 」に設定してみてください。そうすれば、オペレータがログステートの後に 「done」をクリックしなくても、ビヘイビアが完了するまで実行できるようになります。

この後、ステートの実装についてのより詳細な議論については、[例2](docs/example2.md)へ続けてください。
