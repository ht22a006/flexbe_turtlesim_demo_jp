
# RotateTurtleState と Userdata

純粋なステートマシンの概念を拡張するFlexBEの2つの重要な部分は以下の通りです。
* 1) HFSMへのビヘイビアの合成
* 2) あるステートから別のステートへ渡すことができる`userdata`。

例えば、[`RotateTurtleState`](../flexbe_turtlesim_demo_flexbe_states/flexbe_turtlesim_demo_flexbe_states/rotate_turtle_state.py) は `userdata` を使って目標の角度を定義します。

より単純なビヘイビアの例から議論を始め、`FlexBE Turtlesim Demonstration`の「Rotate」遷移の詳細に戻ります。

----

### `Turtlesim Rotation State Behavior`

`FlexBE Turtlesim Demo`で使用している`Turtlesim Input State Behavior`サブビヘイビアとは別に、もっと簡単な`Turtlesim Rotation State Behavior`ビヘイビアを用意しました。

まずは、その最初を説明することから始めましょう。このビヘイビアを読み込んで実行しても構いません。

FlexBEの各ステートは指定された`input_keys`に従ってデータを受け取ることができます。
これらのキーの名前は、ステートレベルで別の名前にリマップすることができます。

例えば、 `RotateTurtleState` の実装では `angle` という入力キーと `duration` という出力キーが指定され、下流のステートに渡されます。
```Python
class RotateTurtleState(EventState):
    """
    ...
    引数
    -- timeout              最大許容時間（秒）
    -- action_topic         呼び出すアクションの名前

    結果
    <= rotation_complete    いくつかの食器だけが洗浄された。（Only a few dishes have been cleaned.意味がわからない。何かのシャレ？）
    <= failed               何らかの理由で失敗した。
    <= canceled             ユーザーが完了前にキャンセルした。
    <= timeout              アクションがタイムアウトした。

    ユーザデータ
    ># angle    float       回転角度の指定 (度) (入力)
    #> duration float       回転完了までの時間(秒) (出力)
    """

    def __init__(self, timeout, action_topic="/turtle1/rotate_absolute"):
        # 基本的な説明は example_state.py を参照
        super().__init__(outcomes=['rotation_complete', 'failed', 'canceled', 'timeout'],
                         input_keys=['angle'],
                         output_keys=['duration'])

        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = action_topic

        # ビヘイビアを構築する際にアクションクライアントを作成する。
        # プロキシクライアントを使用することで、結果とステータスに非同期でアクセスできるようになり、
        # ビヘイビアでこのステートが何度使用されても、使用されるクライアントは1つだけになる。
        ProxyActionClient.initialize(RotateTurtleState._node)

        self._client = ProxyActionClient({self._topic: RotateAbsolute},
                                         wait_duration=0.0)  # 必要なクライアントをdictとして渡す (topic: type)

        # アクションクライアントがアクションゴールの送信に失敗することがある。
        self._error = False
        self._return = None  # オペレータによって結果がブロックされた場合に戻り値を保持する。
        self._start_time = None

```

内部的には、状態の実装は `userdata.angle` を使用して、
基本的な `dict` オブジェクトの機能を拡張した FlexBE コア [`userdata.py` クラス](https://github.com/flexbe/flexbe_behavior_engine/flexbe_core/flexbe_core/userdata.py) を使用して保存されたデータにアクセスします。

`Turtlesim Rotation State Behavior` ビヘイビアでは、FlexBE UI Dashboard で `userdata` を `angle_degrees` と定義すます。
`RotateTurtleState` エディターにおいて、必要な `angle` キーが、リマップされた `angle_degrees` キー値を使用するように指定します。
<p float="center">
  <img src="../img/rotate_state_userdata.png" alt="ステートマシンレベルのuserdata." width="45%">
  <img src="../img/data_flow_graph.png" alt="エディタのデータフロービュー。" width="45%">
</p>

右の図には`Data Flow`ビューも示されており、ビヘイビアの設計者は`userdata`がどのようにステートマシンを通過するかを見ることができます。 
一度定義された `userdata` のキーと値のペアは、ステートマシンの寿命が尽きるまで持続します。

これでステートが実行されると、タートルは `Turtlesim` が提供する [`RotateAbsolute`](https://docs.ros2.org/foxy/api/turtlesim/action/RotateAbsolute.html) アクションで要求される `radians` に変換した後、定義されたキーの値だけ回転します。

> 注：通常、データの受け渡しには一貫した慣習に従うことをお勧めします。ROSは慣例により、角度に`ラジアン`を使用します。 
> ここでは、データ変換の説明とUIでのオペレータの利便性のために`degrees`を選択しました。

`userdata` は各 FlexBE ステートの標準的な `on_enter`、`execute`、`on_exit` メソッドに渡されます。
ここではデータを検証し、`RotateAbsolute` アクションの `Goal` リクエストを作成するために使用します。

```python
def on_enter(self, userdata):

    # 以前の状態実行が失敗している可能性があるため、エラー状態を必ずリセットすること
    self._error = False
    self._return = None

    if 'angle' not in userdata:
        self._error = True
        Logger.logwarn("RotateTurtleState requires userdata.angle key!")
        return

    # 回転時間出力を設定するために開始時間を記録する
    self._start_time = self._node.get_clock().now()

    goal = RotateAbsolute.Goal()

    if isinstance(userdata.angle, (float, int)):
        goal.theta = (userdata.angle * math.pi) / 180  # ラジアンへ変換
    else:
        self._error = True
        Logger.logwarn("Input is %s. Expects an int or a float.", type(userdata.angle).__name__)

    # ゴールを送信
    try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
    except Exception as e:
        # ステートの失敗が必ずしもビヘイビアの失敗を引き起こすとは限らないため, 
        # エラーではなく警告のみを表示することが推奨さる.
        # エラーログを追加する前に改行を使用すると、オペレータはGUIで詳細を折りたたむことができる。
        Logger.logwarn('Failed to send the RotateAbsolute command:\n%s' % str(e))
        self._error = True
```

次に `execute` メソッドにおいて、成功した結果を監視し、送信する `userdata.duration` の値を設定します。
この値は、上記のステート編集ウィンドウで定義したリマッピングに従って、グローバルな `userdata` インスタンスに格納されます。

```python
def execute(self, userdata):
    # このステートがアクティブな間は、アクションが終了したかどうかをチェックし、結果を評価する。

    # クライアントがゴールの送信に失敗したかどうかをチェックする。
    if self._error:
        return 'failed'

    if self._return is not None:
        # 自律性レベルによって遷移が待たされる場合、事前の結果を返す。
        return self._return

    # アクションが終了したかどうかをチェックする
    if self._client.has_result(self._topic):
        _ = self._client.get_result(self._topic)  # デルタ結果の値はここでは役に立たない
        userdata.duration = self._node.get_clock().now() - self._start_time
        Logger.loginfo('Rotation complete')
        self._return = 'rotation_complete'
        return self._return

    if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
        # ゴール応答をチェックした後にタイムアウトをチェックする
        self._return = 'timeout'
        return 'timeout'

    # アクションがまだ終了していない場合、結果は返されず、ステートはアクティブのままである。
    return None
```

FlexBEの「協調的自律性」の側面を実証するために、次の節では`FlexBE Turtlesim Demonstration`の「Rotate」遷移について議論する。

----

## 「Rotate」 - オペレータ入力による協調的自律性

「Rotate」サブビヘイビアは、FlexBEのいくつかの機能を説明するために使用されます。

#### InputState と協調的自律性

[FlexBE Behavior Engine](https://github.com/flexbe/flexbe_behavior_engine) は、 [`BehaviorInput` アクション](https://github.com/flexbe/flexbe_behavior_engine/flexbe_msgs/action/BehaviorInput.action) インターフェースを通じてオペレータのデータを受け付ける [`InputState`](https://github.com/flexbe/flexbe_behavior_engine/flexbe_states/flexbe_states/input_state.py) を提供します。

さらに、FlexBEは[`flexbe_input`パッケージ](https://github.com/flexbe/flexbe_behavior_engine/flexbe_input)の一部として、PyQtベースのUIウィンドウを持つシンプルなアクションサーバを提供しています。

`ros2 run flexbe_input input_action_server`

FlexBEのオンボードの`InputState` が指定されたタイプのデータを要求すると、UIウィンドウが開き、指定されたテキストをユーザに入力するよう促し、ユーザの入力を待ちます。
ユーザが `Enter/Return` を押すか `Submit` ボタンをクリックすると、データはシリアライズされ、アクションの結果の一部としてバイト列データとして `InputState` に返されます。

> 注意: `InputState` は `pickle` モジュールを使用しているため、Pickle マニュアルの以下の警告が適用されます。

> 警告 pickle モジュールは誤ったデータや悪意を持って作成されたデータに対して安全ではありません。
> 信頼されていない、あるいは認証されていないソースから受け取ったデータは絶対に復元 (unpickle) しないでください。

#### ビヘイビアコンテナによるサブビヘイビア 

`FlexBE Turtlesim Demo`のステートマシンでは、「Rotate」と書かれたコンテナ自体が単純なステートマシンです。
つまり、これは[「Eight」](eight_loop.md)のような単なるステートマシンではなく、実際には別のビヘイビア[`Turtlesim Input State Behavior`](../flexbe_turtlesim_demo_flexbe_behaviors/flexbe_turtlesim_demo_flexbe_behaviors/turtlesim_input_state_behavior_sm.py) であり、`FlexBE Turtlesim Demo` ビヘイビアとは独立してFlexBEにロードして実行することができます。

<p float="center">
  <img src="../img/flexbe_input_userdata.png" alt="InputStateを使ったTurtlesim Input State Behaviorのデータフロー。" width="45%">
  <img src="../img/input_ui_running.png" alt="input_action_server からの入力ユーザインタフェイスのポップアップ。" width="45%">
</p>

InputState`の設定では、次のようにしています。
  * ユーザーから1つの数字を要求するために、結果タイプ1 ([`BehaviorInput.Goal.REQUEST_FLOAT`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_msgs/action/BehaviorInput.action)) を指定します。 
  * ユーザーインターフェースのプロンプトメッセージを指定します。
  * `input_action_server`が利用可能になるまでのタイムアウト値を指定します。
  * 出力`userdata`キーのマッピングを指定します。

> 注：float型については、小数を除いた整数値も受け付けます。

> 注: `InputState`の`timeout` はアクションサーバーが利用可能になるのを待つことを意味します。
> システムはオペレータが応答するまで無期限に待ちます。


「Rotate」を要求した後にサブビヘイビアを実行すると、`input_action_server`は右端の画像のようなダイアログをポップアップし、指定されたプロンプトとアクションゴールで指定された結果タイプのプロンプト（この場合は`float`に対して`1`）を表示します。

値を送信した後、もし「Low」自律性で動作していれば、オペレータは「received」遷移を確認する必要があります。

#### ROS 2 アクションインタフェース

`InputState`と`RotateTurtleState`はどちらもROS [action](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)のインターフェイスを利用しています。

これらはFlexBE内で外部ノードとやりとりするのに適した方法です。

`InputState`は、[`BehaviorInput`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_msgs/action/BehaviorInput.action) サーバインターフェースを提供する `input_action_server` と対話するアクションクライアントを使用しています。

`turtlesim`ノードは[`RotateAbsolute`](https://docs.ros2.org/foxy/api/turtlesim/action/RotateAbsolute.html)アクションサーバーインターフェイスを提供しています。

これらのFlexBEステートはどちらも[`ProxyActionClient`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_core/flexbe_core/proxy/proxy_action_client.py)を使用します。これは各ステートクラスの`__init__`メソッドで設定されます。

```python
        self._client = ProxyActionClient({self._topic: RotateAbsolute},
                                         wait_duration=0.0)  # 必要なクライアントをdictとして渡す (topic: type)
```
FlexBEは「プロキシ」を使用して、ビヘイビアのすべてのステートに単一のインターフェイスを提供します。 これにより、必要となる独立した通信チャネルの数を減らすことができます。

通常、コンストラクタで `_client` を作成し、`on_enter` と `execute` で必要に応じてプロキシインスタンスを使用する。

ステートがゴールより先に終了した場合（例えば、オペレータが先取りを要求した場合）、通常、アクションゴールを`on_exit`でキャンセルします。

```python
    def on_exit(self, userdata):
        # このステートから離れるときは、アクションが実行されていないことを確認する。
        # アクションがまだアクティブである状況とは、例えば、オペレータが手動で結果をトリガーする場合である。

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
```
----

この例では、協調的自律性においてオンボードビヘイビアにオペレータデータを提供するための`InputState`の使用、より複雑なビヘイビアを定義するためのビヘイビアコンポジションの使用、より計算集約的な外部ノードと対話するための主なアプローチとしてのROS 2 `action`インターフェースの使用について説明しました。

[概要に戻ります](../README.md#selectable-transitions)