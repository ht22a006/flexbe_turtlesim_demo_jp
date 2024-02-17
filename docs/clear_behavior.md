# Turtlesim Demo クリアビヘイビア

「クリア」サブビヘイビアは、メイン エディター ビューと同様に、一連のステートによって実装されています。

<p float="center">
  <img src="../img/editor_view.png" alt="State machine editor view" width="45%">
</p>

この議論は、あなたがすでに[「Home」](home_behavior.md)の議論に目を通していることを前提としています。

「Clear」の遷移は、まず「ClearLog」[LogState](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_states/flexbe_states/log_state.py) を呼び出して `"Clear turtlesim window ..."`のテキストを表示し、次に [`ClearTurtlesimState`](../flexbe_turtlesim_demo_flexbe_states/flexbe_turtlesim_demo_flexbe_states/clear_turtlesim_state.py) の「ClearWindow」ステートインスタンスに遷移します。 
`ClearTurtlesimState`が戻った後、システムは「Operator」決定ステートに戻るか、「ClearFailed」`LogState`に遷移して問題をオペレータに通知します。

`ClearTurtlesimState`と[`TeleportAbosoluteState`](../flexbe_turtlesim_demo_flexbe_states/flexbe_turtlesim_demo_flexbe_states/teleport_absolute_state.py)の主な違いは、`ClearTurtlesimState`が *待たせる（blocking）* サービスコールを使用することです。


```python
    def on_enter(self, userdata):
        """
        ステートがアクティブになったときに呼び出す。

        この例では、上流のステートから渡されたuserdataは使用しない。

        つまり、他のステートからこのステートへの遷移が行われる。
        """
        self._start_time = self._node.get_clock().now()
        self._return = None  # 完了フラグをリセット
        self._srv_result = None
        self._service_called = False
        try:
            if self._srv.is_available(self._srv_topic, wait_duration=0.0):
                Logger.localinfo(f"{self._name}: Service {self._srv_topic} is available ...")
                self._do_service_call()
            else:
                Logger.logwarn(f"{self._name}: Service {self._srv_topic} is not yet available ...")
        except Exception as exc:
            Logger.logerr(f"{self._name}: Service {self._srv_topic} exception {type(exc)} - {str(exc)}")

    def _do_service_call(self):
        """同期の待たせる呼び出しを使ってサービス・コールを行う。"""
        try:
            Logger.localinfo(f"{self._name}: Calling service {self._srv_topic} ...")
            self._service_called = True
            self._srv_result = self._srv.call(self._srv_topic, self._srv_request, wait_duration=0.0)
        except Exception as e:
            Logger.logerr(f"{self._name}: Service {self._srv_topic} exception {type(e)} - {str(e)}")
            self._srv_result = None
```

FlexBEでは、待たせる呼び出し（blocking call）の使用は推奨されておらず、ほとんどの場合、非同期モデルを好むべきです。

典型的なステートマシンのモデルでは、`on_enter`、`execute`、`on_exit` の呼び出しは、システムの望ましい更新頻度に比べて比較的高速であると仮定しています。 
これは、ステートを `ConcurrencyContainer`（[Examples](examples.md)で説明します）に埋め込むことができる場合に特に重要です。

とはいえ、ここで示したようにブロッキング呼び出しは可能であり、場合によっては適切なこともあります。
ただ、これはステートマシンが単一のスレッドで実行されるため、他のすべてのステートのタイミングに影響を与えることに注意してください。

----

この例では、FlexBE内で待たせる同期サービスコールを使用することを示しました。
待たせないの非同期サービスコールとの比較については、[「Home」](home_behavior.md) の説明を参照してください。

[概要に戻ります](../README.md#selectable-transitions)

