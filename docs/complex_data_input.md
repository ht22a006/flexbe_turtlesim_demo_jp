# 複雑なデータ入力

Turtlesimのデモンストレーションの（サブ）ビヘイビアの中には、オペレータの入力を使用するものがあります。

このために、[`input_action_server`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_input/flexbe_input/input_action_server.py) を使って、`flexbe_behavior_engine` リポジトリの `flexbe_input` パッケージの一部としてシンプルなポップアップダイアログウィンドウを提供します。

この簡単なデモンストレーションは、数値や数値のリスト／タプルといった限られたプリミティブ入力に対する基本的な機能を提供することを目的としています。

より高度なデータ型をサポートするために、必要に応じてより複雑なデータ構造のユーザインターフェイスを開発することが奨励されます。

`InputState` は `pickle` モジュールを使用しているため、Pickle マニュアルの以下の警告が適用されます。

> 警告 pickle モジュールは誤ったデータや悪意を持って作成されたデータに対して安全ではありません。
> 信頼されていない、あるいは認証されていないソースから受け取ったデータは絶対に復元 (unpickle) しないでください。

*`InputState`を使用する場合、信頼できないデータからネットワークを保護するのはユーザー次第です。*

ROS 2 のメッセージは `pickle.dumps` を使用してシリアライズし、 `InputState` に渡すことができます。
例えば、以下のコードではアクションリクエストに応答して FlexBE に送信する `Pose` メッセージを作成しています。

```python
import ast
import pickle

from geometry_msgs.msg import Pose
from flexbe_msgs.action import BehaviorInput


# カスタムデザイン側（UIまたはユーザが開発したアクションサーバノード）
p = Pose()
p.position.x = 42.

result = BehaviorInput.Result()
result.data = str(pickle.dumps(p))  # バイト列として送信するためのデータをフォーマット

# 入力ステート側
input_data = ast.literal_eval(result.data)  # 文字列をバイト配列に変換
response_data = pickle.loads(input_data)  # Pythonオブジェクトにデータをロード

print(f" respose =?= original : {p == response_data}")  # 変換を検証
```

より複雑なタイプを扱うために、[`input_action_server`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_input/flexbe_input/input_action_server.py)の独自のバリエーションを作成することをお勧めします。


