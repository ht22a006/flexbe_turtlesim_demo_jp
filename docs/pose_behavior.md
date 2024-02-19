
# Pose ビヘイビア

「Pose」遷移では、[「Rotate」](rotate_behavior.md)で説明した`InputState`と、[「Home」](home_behavior.md)で最初に説明した`TeleportAbsoluteState`を含む3つのステートを持つ`StateMachine`コンテナを使用します。

`InputState` は `userdata` `data` キーをリマップして、 `TeleportAbsoluteState` が使用する `pose` キーを提供します。

<p float="center">
  <img src="../img/pose_behavior.png" alt="Poseビヘイビアのデータの流れ。" width="45%">
  <img src="../img/pose_input.png" alt="数値のリストとしてPose入力。" width="45%">
</p>

`InputState`の設定では、次のようにしています。
  * 3つの数字の `list` (または `tuple`) をユーザーに要求するために、 結果タイプ 3 ([`BehaviorInput.Goal.REQUEST_3D`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_msgs/action/BehaviorInput.action)) を指定します。
  * ユーザインタフェースのプロンプトメッセージを指定します。
  * `input_action_server` が利用可能になるまでのタイムアウト値を指定します。
  * 出力の`userdata` のキーマッピング（例：この場合は `pose`）を指定します。

> 注：2D、3D、4Dのリクエストタイプでは、`list`(例: '[1., 2, 3]')、`tuple`(例: '(1., 2, 3)')、
> または適切な長さのカンマで区切られた数字列(例: '1., 2, 3')をUI上で入力することができます。

> 注: `InputState` `timeout` はアクションサーバが利用可能になるのを待つことです。
> システムはオペレータの応答を無期限に待ちます。

`TeleportAbsoluteState`は`userdata`からポーズのデータを抽出し、[「Home」](home_behavior.md)で説明されているように、待たない (non-blocking)のサービスコールを行います。

```python
        if 'pose' in userdata and isinstance(userdata.pose, (list, tuple)):
            try:
                self._srv_request.x = float(userdata.pose[0])
                self._srv_request.y = float(userdata.pose[1])
                self._srv_request.theta = 0.0
                if len(userdata.pose) == 3:
                    # 角度の設定は任意
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
```
----

この例では、協調自律性において、より複雑なオペレータのデータをオンボードのビヘイビアに提供するための`InputState`の使用について説明しました。
個々のステートの詳細については[「Rotate」](rotate_behavior.md)と[「Home」](home_behavior.md)を、`StateMachine`コンテナの詳細については[「Eight」](eight_loop.md)を参照してください。


[概要に戻ります](../README.md#selectable-transitions)
