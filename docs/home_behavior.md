# Turtlesim Demo ホーム ビヘイビア

「ホーム」サブビヘイビアは、以下のような一連のステートによって実装されます。

<p float="center">
  <img src="../img/home_editor_view.png" alt="'Home' behavior." width="45%">
</p>

このビヘイビアは、画面をクリアした後に`FlexBE Turtlesim Demo`ビヘイビアに最初に入ったときに最初に呼び出されます。 
その後、ビヘイビアはモニタリングビューに表示される「Operator」決定ステートから「Home」遷移を選択することで呼び出されます。
アクティブなステートは、まずメッセージテキスト`"Go to home position"`で「GoHome」[`LogState`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_states/flexbe_states/log_state.py)に遷移し、次に[`TeleportAbsoluteState`](../flexbe_turtlesim_demo_flexbe_states/flexbe_turtlesim_demo_flexbe_states/teleport_absolute_state.py)
ステート実装の「Home」ステートのインスタンスに遷移します。 
その結果は、「AtHome」または「ServiceCallFailed」ステートによってログ出力し、システムは「Operator」決定ステートに遷移します。

以下のコードに示すように、`TeleportAbsoluteState` は、`turtlesim` ノードが提供する[`TeleportAbsolute`](https://docs.ros2.org/latest/api/turtlesim/srv/TeleportAbsolute.html) サービスへの FlexBE インターフェースを提供します。
このノードは `__init__` メソッド呼び出しの入力引数として、または `userdata` として位置を受け取ることができます。
`userdata`については["Rotate"](rotate_behavior.md)を参照してください。

`__init__`メソッド コンストラクタは、実際の呼び出しを処理するための[`ProxyServiceCaller`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_core/flexbe_core/proxy/proxy_service_caller.py)インスタンスを設定します。
FlexBE では、複数のステートがノードへの単一のアクセスポイントを共有して、他のノードへのインターフェースのパブリッシュ、サブスクライブ、呼び出しを行えるようにするために、多くの *Proxy* インターフェースを使用します。
オンボードシステムは外部ノードへのアクセスポイントとして単一のROS `node` を維持します。

```python 
from rclpy.duration import Duration
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from turtlesim.srv import TeleportAbsolute


class TeleportAbsoluteState(EventState):
    """
    このステートは、TeleportAbsoluteサービスを使用してTurtlesimタートルをテレポートする。

    引数
    -- turtle_name  string  タートルの名前 (デフォルト: `turtle1`)
    -- x            float   x 位置 (デフォルト: 0.0)
    -- y            float   y 位置 (デフォルト: 0.0)
    -- theta        float   ヨーの向き角度（ラジアン）（デフォルト：0.0）
    -- call_timeout float   完了までのタイムアウト時間（デフォルト：3.0 秒）
    -- wait_timeout float   サービスが利用可能になるまでの待ち時間（デフォルト：3.0秒）。
    -- service_name string  サービス名（デフォルト：`teleport_absolute`）。

    結果
    <= done             サービスコールが期待通りの結果を返した
    <= failed           サービスコールに失敗した
    <= call_timeout     サービスコールはタイムリーに結果を返さなかった
    <= unavailable      サービスが利用できない

    ユーザデータ
    ># pose     float[] 2要素(x,y)または3要素(x,y,theta_radians)の数値リスト
    """

    def __init__(self, turtle_name='turtle1', x=0.0, y=0.0, theta=0.0,
                 call_timeout=3.0, wait_timeout=3.0, service_name='teleport_absolute'):
        """親クラスEventStateのコンストラクタを呼び出して、outcomes、input_keys、output_keysを宣言する。"""

        super().__init__(outcomes=['done', 'failed', 'call_timeout', 'unavailable'],
                         input_keys=['pose'])

        ProxyServiceCaller.initialize(TeleportAbsoluteState._node)

        # 後で使用するためにステートパラメータを保存する。
        self._call_timeout = Duration(seconds=call_timeout)
        self._wait_timeout = Duration(seconds=wait_timeout)

        # コンストラクタは、実際にビヘイビアを開始するときではなく、ステートマシンを構築するときに呼び出される。
        # したがって、ここでは開始時間を保存できず、後で保存することになる。
        self._start_time = None
        self._return = None  # 結果を追跡することで、遷移が待たされているかどうかを検出できる。
        self._service_called = False

        self._srv_topic = f'/{turtle_name}/{service_name}'
        self._srv_result = None

        self._srv_request = TeleportAbsolute.Request()
        self._srv_request.x = x
        self._srv_request.y = y
        self._srv_request.theta = theta

        self._error = None

        # ここでプロキシを設定するが、まだサービスを待ってはいけない。
        self._srv = ProxyServiceCaller({self._srv_topic: TeleportAbsolute}, wait_duration=0.0)
```

上記のステートの実装を与えると、`FlexBE Turtlesim Demo` ビヘイビアはこのクラスのインスタンスを定義し、[`flexbe_turtlesim_demo_sm.py`](../flexbe_turtlesim_demo_flexbe_behaviors/flexbe_turtlesim_demo_flexbe_behaviors/flexbe_turtlesim_demo_sm.py) の最上位のステートマシンに追加します。

以下に `flexbe_turtlesim_demo_sm.py` のビヘイビアの実装の一部を示します。
`TeleportAbsoluteState()`コンストラクタに代入された値は、上のFlexBEエディタウィンドウから取得したものです。

```python
_state_machine = OperatableStateMachine(outcomes=['finished'])

with _state_machine:
    # x:178 y:77
    OperatableStateMachine.add('Home',
                                TeleportAbsoluteState(turtle_name='turtle1', x=5.544, y=5.544, theta=0.0,
                                                        call_timeout=3.0, wait_timeout=3.0,
                                                        service_name='teleport_absolute'),
                                transitions={'done': 'AtHome', 'failed': 'ServiceCallFailed',
                                            'call_timeout': 'ServiceCallFailed',
                                            'unavailable': 'ServiceCallFailed'},
                                autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'call_timeout': Autonomy.Off,
                                            'unavailable': Autonomy.Off},
                                remapping={'pose': 'home'})

    # x:651 y:133
    OperatableStateMachine.add('Operator',
                                OperatorDecisionState(outcomes=["Home", "Eight", "Quit", "Clear", "Rotate", "Pose"],
                                                        hint="Eight", suggestion="Eight"),
                                transitions={'Home': 'GoHome', 'Eight': 'EightMove', 'Quit': 'finished',
                                            'Clear': 'ClearLog', 'Rotate': 'Turtlesim Input State Behavior',
                                            'Pose': 'GoTo'},
                                autonomy={'Home': Autonomy.Full, 'Eight': Autonomy.High,
                                            'Quit': Autonomy.Full, 'Clear': Autonomy.Full,
                                            'Rotate': Autonomy.Full, 'Pose': Autonomy.Full})


    # x:461 y:80
    OperatableStateMachine.add('AtHome',
                                LogState(text="Turtle is home!", severity=Logger.REPORT_HINT),
                                transitions={'done': 'Operator'},
                                autonomy={'done': Autonomy.Off})


```

この議論では、高レベルの概要に留まることにします。ステートのライフサイクルの詳細については、[例](examples.md)を参照してください。

「Home」ステートがアクティブになると、 `TeleportAbsoluteState` クラスの `on_enter`メソッドが呼び出されます。
システムが `userdata` を使用している場合は、目的のポーズが抽出されます。 今回のケースでは、`pose`という名前をビヘイビアダッシュボードで定義された`home`という`userdata`にリマップしています。 `userdata`については[Rotate](rotate_behavior.md)を参照してください。

この `userdata` が提供されない場合、ステートは上記の `flexbe_turtlebot_demo_sm` コードで作成時に定義されたパラメータをデフォルトで使用します。

ステートインスタンスは `self._start_time` を記録し、利用可能であれば非同期（待たない）サービスコールを使用してサービスコールが起動されます（["Clear"](clear_behavior.md)でのディスカッションとは対照的に）。 
例外が発生した場合、ステートの`self._return` は `failed` としてマークされます。

```python 
    def on_enter(self, userdata):
        """
        ステートがアクティブになったときにこのメソッドを呼び出す。

        つまり、別のステートからこのステートへの遷移が行われる。
        """

        if 'pose' in userdata and isinstance(userdata.pose, (list, tuple)):
            try:
                self._srv_request.x = float(userdata.pose[0])
                self._srv_request.y = float(userdata.pose[1])
                self._srv_request.theta = 0.0
                if len(userdata.pose) == 3:
                    # 角度設定は任意
                    self._srv_request.theta = float(userdata.pose[2])

                Logger.localinfo(f"Using position = ({self._srv_request.x:.3f}, {self._srv_request.y:.3f}), "
                                 f"angle={self._srv_request.theta:.3f} radians from userdata")

            except Exception as exc:  # pylint: disable=W0703
                Logger.logwarn(f"{self._name}: Invalid pose userdata {userdata.pose} - "
                               f"needs list of 2 or 3 numbers!\n  {type(exc)} - {exc}")
                self._return = 'failed'
                return
        else:
            Logger.localinfo(f"Using position = ({self._srv_request.x:.3f}, {self._srv_request.y:.3f}), "
                             f"angle={self._srv_request.theta:.3f} radians")

        self._start_time = self._node.get_clock().now()
        self._return = None  # 完了フラグをリセット
        self._service_called = False
        try:
            if self._srv.is_available(self._srv_topic, wait_duration=0.0):
                self._do_service_call()
            else:
                Logger.logwarn(f"{self._name}: Service {self._srv_topic} is not yet available ...")
        except Exception as exc:
            Logger.logerr(f"{self._name}: Service {self._srv_topic} exception {type(exc)} - {str(exc)}")
            self._return = 'failed'

    def _do_service_call(self):
        """非同期で待たずにサービスコールを行う。"""
        try:
            Logger.localinfo(f"{self._name}: Calling service {self._srv_topic} ...")
            self._srv_result = self._srv.call_async(self._srv_topic, self._srv_request, wait_duration=0.0)
            self._start_time = self._node.get_clock().now()  # 呼び出しタイムアウトのタイマをリセット
            self._service_called = True
        except Exception as exc:
            Logger.logerr(f"{self._name}: Service {self._srv_topic} exception {type(exc)} - {str(exc)}")
            raise exc

```

`on_enter` メソッドは、ステートマシンの中で遷移した後にステートのアクティブになったときに一度だけ呼び出されます。
それ以降、FlexBE ビヘイビアの実行部は、`None`以外が返されるまで、周期的に`execute`メソッドを呼び出します。

サービスが呼び出された場合、指定されたタイムアウト時間が経過するまで結果を待ちます。
サービスが `on_enter` で利用可能でなかった場合は、指定されたタイムアウト時間まで利用可能になったときに呼び出します。

```python
    def execute(self, userdata):
        """
        ステートがアクティブである間、このメソッドを周期的に実行する。

        結果が返されない場合、ステートはアクティブのまま。
        """
        if self._return:
            # ステートが完了したら、自律性レベルによって待たされなければならない
            return self._return

        if self._service_called:
            # 結果を待つ
            # これを待たないやり方で行う
            if self._srv.done(self._srv_topic):
                _ = self._srv.result(self._srv_topic)  # 空の結果を取得するが、ここでは成功しているかどうか何も確認しない
                self._return = 'done'
            else:

                if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._call_timeout.nanoseconds:
                    # 時間内に呼び出しを返さななかった
                    self._return = 'call_timeout'
                    Logger.logerr(f"{self._name}: Service {self._srv_topic} call timed out!")
        else:
            # 待たないやり方でサービスが利用可能になるのを待つ
            if self._srv.is_available(self._srv_topic, wait_duration=0.0):
                Logger.localinfo(f"{self._name}: Service {self._srv_topic} is now available - making service call!")
                self._do_service_call()
                # 次のexecuteの呼び出しで結果を処理する（そのため、若干の遅延が発生する）
            else:
                if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._wait_timeout.nanoseconds:
                    # 時間内にサービスが有効にならなかった
                    self._return = 'unavailable'
                    Logger.logerr(f"{self._name}: Service {self._srv_topic} is unavailable!")

        return self._return
```

この例では、FlexBE内で非同期サービスコールを使用することを示しました。
ブロッキングサービスコールとの比較については、["Clear"](clear_behavior.md) を参照してください。

[概要に戻ります](../README.md#selectable-transitions)

