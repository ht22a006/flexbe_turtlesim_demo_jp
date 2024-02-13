# 例4 - ビヘイビア構成 - ビヘイビアを他のビヘイビアで使用する

`例4` のビヘイビアは、複数の [`ConcurrencyContainer`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_core/flexbe_core/core/concurrency_container.py) ステートと複数の `ExampleState` を持つ `Behavior` コンテナの中に、`例3` のビヘイビアを含む 3 層の HFSM を構築します。

<p float="center">
  <img src="../img/example4_editor.png" alt="例4 エディタビュー。" width="40%">
  <img src="../img/example3_top_level_sm.png" alt="例3 最上位のステートマシン。" width="40%">
</p>

「Add Behavior」を使用すると、通常の「Load Behavior」ダイアログが開きます。
このダイアログで、ビヘイビア コンテナ ステートのローカル ステート名を変更できます。

上の一番左の画像に示されているステート エディタ ビューを使って、ステートマシンに定義されているステートマシン パラメータ（[「例3」](example3.md)で定義されているもの）を変更することができます。
この例では、`waiting_time_a`を修正しました。

既存のビヘイビアを組み合わせて、より複雑なビヘイビアを構築できるこの機能は、FlexBEの強力な特徴です。

この3層のHFSMの例は、説明のための比較的些細なものですが、FlexBEはビヘイビア合成を使用して、はるかに複雑なビヘイビアを開発するために使用されています。

いくつかの例については、以下の文献を参照してください。

- Stefan Kohlbrecher et al. ["A Comprehensive Software Framework for Complex Locomotion and Manipulation Tasks Applicable to Different Types of Humanoid Robots."](http://dx.doi.org/10.3389/frobt.2016.00031) Frontiers in Robotics and AI 3 (2016): 31.

- Alberto Romay et al., [“Collaborative autonomy between high-level behaviors and human operators for remote manipulation tasks using different humanoid robots,”](http://dx.doi.org/10.1002/rob.21671) Journal of Field Robotics, September 2016.



