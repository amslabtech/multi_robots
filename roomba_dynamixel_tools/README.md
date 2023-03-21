# roomba_dynamixel_tools

## インストールとビルド
事前にDynamixel SDK,Dynamixel Workbench,dynamixel_workbench_msgsをインストールしておく必要がある

* [Dynamixel SDK](https://github.com/TakuKarasawa/DynamixelSDK)
```
git clone https://github.com/TakuKarasawa/DynamixelSDK.git
```
* [Dynamixel Workbench](https://github.com/TakuKarasawa/dynamixel_workbench)
```
git clone https://github.com/TakuKarasawa/dynamixel_workbench.git
```
* [dynamixel_workbench_msgs](https://github.com/TakuKarasawa/dynamixel_workbench_msgs)
```
git clone https://github.com/TakuKarasawa/dynamixel_workbench_msgs.git
```

## Dynamixel MX-28AR(Protocol 2.0)の起動方法
### 1. ポート権限を得る(*は番号), udevの設定をしている場合は不要([udevの設定](https://github.com/amslabtech/multi_robots/tree/master/multi_robots/udevs))
```
sudo chmod a+rw /dev/ttyUSB*
```
<br>

### 2. Dynamixel MX-28AR(Protocol 2.0)が検知できるか確認(*は番号)
```
rosrun dynamixel_workbench_controllers find_dynamixel /dev/ttyUSB*
```
このときに，以下のようなテキストが表示されるので，"id"を記録しておく(下図だと"1")
```
[ INFO] [1544589715.841211668]: Succeed to init(9600)
[ INFO] [1544589715.841236741]: Wait for scanning...
[ INFO] [1544589737.539083688]: Find 0 Dynamixels
[ INFO] [1544589737.539526809]: Succeed to init(57600)
[ INFO] [1544589737.539570059]: Wait for scanning...
[ INFO] [1544589755.441019922]: Find 1 Dynamixels
[ INFO] [1544589755.441086482]: id : 1, model name : MX-28-2
[ INFO] [1544589755.441504892]: Succeed to init(115200)
[ INFO] [1544589755.441548969]: Wait for scanning...
[ INFO] [1544589773.031677244]: Find 0 Dynamixels
[ INFO] [1544589773.032153380]: Succeed to init(1000000)
[ INFO] [1544589773.032178580]: Wait for scanning...
[ INFO] [1544589790.291943770]: Find 0 Dynamixels
[ INFO] [1544589790.292404604]: Succeed to init(2000000)
[ INFO] [1544589790.292418207]: Wait for scanning...
[ INFO] [1544589807.530702991]: Find 0 Dynamixels
[ INFO] [1544589807.531286252]: Succeed to init(3000000)
[ INFO] [1544589807.531331656]: Wait for scanning...
[ INFO] [1544589824.762803705]: Find 0 Dynamixels
[ INFO] [1544589824.763461821]: Succeed to init(4000000)
[ INFO] [1544589824.763506935]: Wait for scanning...
[ INFO] [1544589841.990120553]: Find 0 Dynamixels
```
<br>

### 3. コントローラを起動
```
roslaunch dynamixel_workbench_contrtollers roomba_joint.launch
```
エラーメッセージが出たら
```
dynamixel-workbench/dynamixel_workbench_controllers/config/roomba*_joint.yaml
```
の"id"が一致していない可能性があるがあるため，先程記録した"id"を一致させる([*はroombaの番号](https://amslab.esa.io/posts/71))\
もしくは，
```
dynamixel-workbench/dynamixel_workbench_controllers/launch/roomba_joint.launch
```
の"usb_port"が一致していない可能性があるので，異なっていたら一致させる
<br>
<br>

### 4. 回転させる

```
roslaunch roomba_dynamixel_controller teleop_roomba_dynamixel.launch
```
作動しない場合は，
```
dynamixel_workbench/dynamixel_workbench_controller/config/roomba*_joint.yaml
```
内の
```
[name]:
  ID: [id]
  [Control_Table_Item]: [value]
  [Control_Table_Item]: [value]
  .
  .
  .
```
の"[name]: と
```
roomba_dynamixel_tools/roomba_dynamixel_controller/config/param/roomba_dynamixel_controller.yaml
```
の"dynamixel_name"が一致していない可能性があるので一致させる

<br>

回転の実行時間を変更させる場合は
```
roomba_dynamixel_tools/roomba_dynamixel_controller/config/param/roomba_dynamixel_controller.yaml
```
内の"execution_time"を変更する

