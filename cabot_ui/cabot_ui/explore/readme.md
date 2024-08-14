## ロボットの準備
- 今のところ、ace22-1で動作確認中
- 必要なもの
  - ロボット本体
  - バッテリー
  - wifiルーター
  - HDMIダミープラグ
1. バッテリーを搭載
2. 電源ボタンを押して起動
3. HDMIのダミープラグが挿さってることを確認
4. 起動後、VNCでロボットに接続（同一wifiネットワーク内で接続）

## .envファイルの編集
OPENAI_API_KEY=<apikey>
を追加する

## ロボット起動関連
1. dockerコンテナの起動

```shell
$ cd cabot-ros2
cabot-ros2$ docker compose -f docker-compose-nuc-production.yaml up driver people-nuc
```

1. jetsonの起動
```shell
$ cd cabot-ros2
cabot-ros2$ ./jetson-launch.sh -v -u cabot -c "D:192.168.1.50:rs1 D:192.168.1.51:rs2 D:192.168.1.52:rs3" -S " rs1:141322250995 rs2:143322070524 rs3:143422072154" -f 30 -p 15 -r 640 -o 3
```

or 

```shell
$ cd /home/cabot/cabot-navigation-exploration/docker/home/ros2_ws/src/scripts
scripts $ ./jetson_cli.sh
```

1. launch.sh起動
```shell
cd /home/cabot/cabot-navigation-exploration
cabot-navigation-exploration$ ./launch.sh -e
```


## GPT-4oでのテキスト生成関連
0. docker containerの起動
```shell
cabot-ros2/cabot-navigation-exploration$ docker compose exec navigation bash
```

1. ロボットのstate control
まずはこれを実行し、ロボットのstateを管理する（手動操作時）
```shell
~/ros2_ws/src $ bash run_state_publisher.sh
[INFO] [1722319096.147677486] [state_input]: State input started
Enter state ([r]unning, [p]aused, [f]inished):
```

2. 周辺環境の説明
```shell
python3 test_image.py --log_dir logs/<exp name> -e [--sim] [--speak]
```
- `/cabot/nav_state` が `running` であるときにのみ説明生成が行われる

3. semantic mapの生成
```shell
python3 test_image.py --log_dir logs/<exp name> -s [--sim]
```
- `/cabot/nav_state` が `running` であるときにのみ説明生成が行われる（周辺環境の説明と同様）

4. 走行可能な方向についての説明生成
```shell
python3 test_image.py --log_dir logs/<exp name> -i [--sim] [--speak]
```
- `/cabot/nav_state` が `paused` であるときにのみ説明生成が行われる


---
まとめて起動する場合
```shell
python3 run_image.py --log_dir logs/<exp name> -e -s -i [--sim] [--speak]
```


## 対話システム関連
0. docker containerの起動
```shell
cabot-ros2/cabot-navigation-exploration$ docker compose exec navigation bash
```

1. 対話サーバー（iPhone -> cabot）の起動
```shell
~/ros2_ws/src $ python3 test_chat_server.py [--use_openai] --log_dir logs/<exp name>
```


## navigation関連
0. docker containerの起動
```shell
cabot-ros2/cabot-navigation-exploration$ docker compose exec navigation bash
```

1. trajectory記録スクリプトの起動
```shell
~/ros2_ws/src $ python3 test_trajectory.py --log_dir logs/<exp name>
```

2. navigationキャンセルスクリプトの起動
```shell
~/ros2_ws/src $ python3 test_explore.py -c
```

3. navigationスクリプトの起動
```shell
~/ros2_ws/src $ python3 test_loop.py -d [-f] -i [-t] --log_dir logs/<exp_name> [--sim] [-a] [-k]
```
- `-d` : distance filter; 距離が近すぎる・遠すぎる目的地を除外
- `-f` : forbidden filter; 禁止エリアを除外。禁止判定は、前・右・左のカメラがとらえた画像をもとに行われる。どれかのカメラにマーカーが写っている、もしくはGPTによる画像説明で「通行不可」判定がなされると、その方向で、ロボットから8m先の場所を中心とする半径4mの円が禁止エリアとして登録される。
- `-t` : trajectory filter; すでに通過した軌跡をなるべく通らないようにする
- `-i` : 画像をもとに禁止エリアを設定する
- `--sim` : シミュレーターでのテスト
- `-a` : 次の目的地を自動で設定する
- `-k` : 次の目的地設定をキーボードから行う。これが有効になっていない場合、`/cabot/user_query` トピックに対して、次の目的地を設定するクエリを送信することで次の目的地を設定することができる。このクエリは、`test_chat_server.py` が起動しているときに、iPhoneから送信することができる。
  - その他のクエリの投げ方
    - curl（`test_chat_server.py`が起動している場合）
      - `curl -X POST http://127.0.0.1:5050/service -H "Content-Type: application/json" -d '{"input":{"text": "木製の展示"}}'` 
      - `curl -X POST http://127.0.0.1:5050/service -H "Content-Type: application/json" -d '{"input":{"text": "right"}}'` 
    - 直接ros2のトピックに対して送信（`std_msgs/String`型のデータを送信）
      - `ros2 topic pub /cabot/user_query std_msgs/String "data: direction;front" --once` 
      -  `ros2 topic pub /cabot/user_query std_msgs/String "data: search;<y>,<x>" --once`