# roomba_visualizer

## Required
- [color_detector_msgs](https://github.com/RenFukatsu/color_detector/tree/master/color_detector_msgs)
- [multi_robots_tf](https://github.com/amslabtech/multi_robots/tree/master/multi_robots_tf)
- [pole_eliminator](https://github.com/amslabtech/multi_robots/tree/master/pole_eliminator)
- [amcl](http://wiki.ros.org/amcl)

## How to Use
1. Hornet上のbag fileを取ってくる．
```
scp -r hornet:~/bagfiles/multi_robot/six_roomba_20210429/replace_ns_bagfiles your/bagfile/place
scp -r hornet:~/bagfiles/multi_robot/six_roomba_20210429/target_pos your/bagfile/place
```

2. [visualize.launch](https://github.com/amslabtech/multi_robots/tree/master/roomba_visualizer/launch/visualize.launch)の5, 7行目のディレクトリを1.で取ってきた自分の環境に合わせる．

3. `roslaunch roomba_visualizer visualize.launch`

## target_posのbag生成について
color_detectorは各ロボットで一つずつ動かす予定の実装のため，ロボットの台数分手元のPCで動かそうとすると，無理が生じる．そのため，`create_target_publish_bag.launch`を使って，target/positionのみをrosbag recordしておく必要がある．
