# Turtlesim Demo 8の字ループ ビヘイビア

「Eight」サブビヘイビアは、FlexBEの`StateMachine`コンテナ型を使って組み込みステートマシンとして実装されています。

新しいステート（基本ステート、コンテナ、ビヘイビア全体を含む）は、エディター・ビューに追加されます。

<p float="center">
  <img src="../img/editor_view_add.png" alt="コンテナを追加しているステートマシンエディタのビュー。" width="45%">
  <img src="../img/timed_cmd_vel.png" alt="「EightMove」ステートマシンコンテナの中のLeftTurnステートのパラメータ。" width="45%">
</p>

ここでは、既存のステートマシンに新しい`State machine`コンテナタイプを追加する例を示します。

与えられた`FlexBE Turtlesim Demo`のビヘイビアでは、コンテナを追加し、その名前を「Container」ではなく、「EightMove」に編集しました。

「EightMove」コンテナの中で、[`TimedCmdVelState`](../docs/flexbe_turtlesim_demo_flexbe_states/flexbe_turtlesim_demo_flexbe_states/timed_cmd_vel_state.py)インスタンスの並びである単純なステートマシンを定義しました。

```python
rom rclpy.duration import Duration
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher

from geometry_msgs.msg import Twist

class TimedCmdVelState(EventState):
    """
    このステートは、パラメータに基づいてオープンループ定数であるTwistコマンドをパブリッシュする。

    -- target_time          float     動作が始まってから経過している必要がある時間。
    -- velocity             float     車体速度（m/s）
    -- rotation_rate        float     角速度 (rad/s)
    -- cmd_topic            string    ロボット速度コマンドのトピック名 (デフォルト: 'cmd_vel')
    -- desired_rate         float     希望するステート更新頻度（デフォルト：50Hz）
    <= done                 与えられた時間が過ぎた。
    """

    def __init__(self, target_time, velocity, rotation_rate, cmd_topic='cmd_vel', desired_rate=50):
        """親クラスのコンストラクタに対応する引数を渡して、outcomes、input_keys、output_keysを宣言する。"""
        super().__init__(desired_rate=desired_rate, outcomes=['done'])

        # 後で使用するためにステートの引を保存する。
        self._target_time = Duration(seconds=target_time)

        # コンストラクタは、実際にビヘイビアを開始するときではなく、ステートマシンを構築するときに呼び出される。
        # したがって、今は開始時刻を保存することはできず、後で保存することになる。
        self._start_time = None

        self._return = None  # outcomeを追跡することで、遷移が待たされているかどうかを検出できる。

        self._twist = Twist()
        self._twist.linear.x = velocity
        self._twist.angular.z = rotation_rate
        self._cmd_topic = cmd_topic

        # FlexBEはパブリッシャー、サブスクライバー、サービス呼び出し元に対して「プロキシ」を使用し、
        # ビヘイビア内のすべてのステートが単一のサブスクリプション/パブリッシャーを共有できるようにする。
        ProxyPublisher.initialize(TimedCmdVelState._node)  # the class must know the behavior node
        self._pub = ProxyPublisher()
        self._pub.createPublisher(cmd_topic, Twist)

    def execute(self, userdata):
        """ステートがアクティブな間、このメソッドを周期的に呼び出す。"""
        if self._return:
            # われわれはステートを完成させたのだから、自律性レベルによって待たされなければならない。
            # ロボットを停止させ、事前の結果を返す。
            if self._cmd_topic:
                self._pub.publish(self._cmd_topic, Twist())

            return self._return

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._target_time.nanoseconds:
            # 正常終了、パブリッシュを繰り返す必要はない
            # 複数のモーションを連鎖させることができるように、
            # 待たされない限り（上記）、わざわざ0コマンドを発行することはない
            self._return = 'done'
            Logger.localinfo(f"{self._name} : returning 'done'")  # For initial debugging
            return 'done'

        # 通常の処理
        if self._cmd_topic:
            Logger.localinfo(f"{self._name} : {self._twist}")  # For initial debugging
            self._pub.publish(self._cmd_topic, self._twist)

        return None

    def on_enter(self, userdata):
        """
        ステートがアクティブになったら、このメソッドを呼び出す。

        つまり、別のステートからこのステートへの遷移が行われる。
        """
        self._start_time = self._node.get_clock().now()
        self._return = None  # 完了フラグをリセット
```

`TimedCmdVelState`は、`on_enter`メソッドで開始時間を保存し、指定されたトピックに、*おおよそ* 希望する更新頻度で、指定されたコマンド速度をパブリッシュします。
繰り返しになりますが、FlexBEはタイミングに関するベストエフォートであり、オペレーティングシステムや他のプロセスに依存します。
FlexBEは保証されたリアルタイムコントローラーでは*ありません*。

[`flexbe_turtlesim_demo_sm.py`](../flexbe_turtlesim_demo_flexbe_behaviors/flexbe_turtlesim_demo_flexbe_behaviors/flexbe_turtlesim_demo_sm.py)にあるステートマシンの実装コードはすべて、ビヘイビアが保存されるときにFlexBE UIによって生成されます。

この場合、以下の断片的なコードに示すように、コードはFlexBE Coreの[`OperatableStateMachine`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_core/flexbe_core/core/operatable_state_machine.py)のインスタンス（`_sm_eightmove_1`と呼ぶ）を作成し、
`TimedCmdVelState`の「LeftTurn」インスタンス（など）を`add`し、`_sm_eightmove_1`インスタンスを最上位のステートマシン`_state_machine`に追加します。 これが FlexBE が HFSM の概念を実装するためのステートの構成方法です。

```python
        # x:975 y:134, x:130 y:365
        _sm_eightmove_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

        with _sm_eightmove_1:

            # x:179 y:211
            OperatableStateMachine.add('LeftTurn',
                                       TimedCmdVelState(target_time=5.77, velocity=0.5, rotation_rate=0.667,
                                                        cmd_topic=cmd_vel, desired_rate=50),
                                       transitions={'done': 'Forward1'},
                                       autonomy={'done': Autonomy.Off})

        with _state_machine:

            # x:654 y:490
            OperatableStateMachine.add('EightMove',
                                       _sm_eightmove_1,
                                       transitions={'finished': 'StopCmd', 'failed': 'FailedEight'},
                                       autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

```

様々なステート --「Forward0」、「LeftTurn」、「Forward1」、「RightTurn」、「Forward2」 -- のパラメータは、基本的な8の字パターンを生成するために計算されました。なお、これらは純粋なオープンループの動きであり、`turtlesim`ノードでスポーンする他のカメを避けたり、境界内にとどまったりするような調整はしていません。

オペレータが「Eight」遷移をクリックすると、または「Full」自律で自動的に選択されると、「EightMove」ステートマシンが最上位（または「ルート」）ステートマシンから見てアクティブな状態になり、
「EightMove」ステートに`on_enter`すると、最初の「Forward0」ステートがアクティブになり、ビヘイビア エンジンによって実行されるアクティブな状態になります。 「EightMove」ステートは「Forward2」ステートが「done」を返すまでアクティブなままであり、その時点で「EightMove」は「finished」という結果を返します。

> 注：「EightMove」の「failed」結果は当初定義されていたが、内部的には接続されていませんでした。
> いずれにせよ、定義された以上、ルートレベルで接続されなければなりません。 これによって将来の修正が可能になります。
> FlexBEは、たとえ一度も行使されなかったとしても、可能性のあるすべてのステートの結果を終了させることを要求します。
> 別の方法として、「EightMove」のエディタビューで、結果ラベルの右側にある赤線のボックスをクリックして、「failed」の結果を削除することもできます。

----

この例では、ステートマシン・コンテナを使ってHFSMを構築することを示しました。

[概要に戻る](../README.md#selectable-transitions)

