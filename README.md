# Makilab Mocap Nocov Ros Driver
- 巻研でMocap Nocovを扱うためのROS１ドライバー
- 既存のソフトウェアのバグ修正、Config設定、動作確認を行ったもの
- Reviser：Sehwa Chun, Hiroki Yokohata

## 動作環境
- NOKOVソフトウェア起動PC：Windows, NOKOV Interface Version 3.4
- NOKOV ROSドライバー起動PC：Ubuntu 20.04, Ros Noetic

## Network Setting
- NOKOVソフトウェア起動PC： 10.1.1.198 (Subnet Mask: 255.255.255.0)
- NOKOV ROSドライバー起動PC: 192.168.70.xxx (Subnet Mask: 255.255.255.0)
- 追加で以下のコマンドをターミナルで起動 (NOKOV系のSubnetと巻研ロボットのサブネットを繋ぐため）
```
sudo ip addr add 10.1.1.100/24 dev enp0s31f6
```
- enp0s31f6はEthernet Deviceの名前、以下のコマンドで確認
```
ip link
```
- ROS IPは192.168.70.xxxで設定

## NOKOVソフトの設定
- こちらを参考　http://wiki.ros.org/mocap_nokov
- 測定したい対象をRigid Bodyでそれぞれ登録
- 名前は（必須！）0,1,2,...の数字一文字で登録すること (必ず０から！）

## Configの設定
- 以下のFormatから何も変えずRigid Bodyの数だけコピペして設定すること
```
nokov@ubuntu:~/catkin_ws/src/mocap_nokov/config$ gedit mocap.yaml
```
```
rigid_bodies:
    '0':
        pose: Robot_1/pose
        pose2d: Robot_1/ground_pose
        odom: Robot_1/Odom
        tf: tf
        child_frame_id: Robot_1/base_link
        parent_frame_id: world
    '1':
        pose: Robot_2/pose
        pose2d: Robot_2/ground_pose
        odom: Robot_2/Odom
        tf: tf
        child_frame_id: Robot_2/base_link
        parent_frame_id: world
nokov_config:
        server_address: "10.1.1.198"
```

## 使用方法
NOKOV ROSドライバー起動PCの方で
```
roslaunch mocap_nokov mocap.launch
```
を叩くと
```
/mocap_node/<tracker_name>/odom
```
にPositionとOrientationが入っている。これを活用すること。
- このOdometryを直接Rvizで出すとカクカクする。
- なので、Sample用のodom2marker.pyを起動すると生値をもとにMarkerを再発行するのでこちらはRvizでヌルヌル動く。
```
rosrun mocap_nokov odom2marker.py
```
- 疑似Perceptionコードやデータ処理を行いたい場合もこのサンプルコードを参考
- Ground PoseやPoseは出てこない。そういうものだと思ってほしい。
- Rvizで出てくる対象の姿勢がおかしかったら、NOKOVソフトでRigid Bodyを登録する基準軸を見直すこと。
- NOKOV関係でわからないことはMotegi Yuichiと相談すること

### 以下はもとのReadme。そのままではうまく動かなったし、目に見えるバグも多少あるので信頼するな
-------------------------


![melodic](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/melodic.yml/badge.svg)
![noetic](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/noetic.yml/badge.svg)

# mocap_nokov
ROS nodes for working with the Nokov motion capture system

## Usage

```
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/NOKOV-MOCAP/mocap_nokov.git
cd ~/catkin_ws
rosdep install --from-paths src -y --ignore-src
catkin_make
source /devel/setup.bash
```

nokov@ubuntu:~/catkin_ws/src/mocap_nokov/config$ gedit mocap.yaml

```
rigid_bodies:
    '1':
        pose: Robot_1/pose
        pose2d: Robot_1/ground_pose
        odom: Robot_1/Odom
        tf: tf
        child_frame_id: Robot_1/base_link
        parent_frame_id: world
    '2':
        pose: Robot_2/pose
        pose2d: Robot_2/ground_pose
        odom: Robot_2/Odom
        tf: tf
        child_frame_id: Robot_2/base_link
        parent_frame_id: world
nokov_config:
        server_address: "10.1.1.198"
```

This is a ROS parameter configuration file for `mocap_nokov`. Here is a description of each parameter:

- `rigid_bodies` : Add rigid body to track.
- `1:` : Asset number of rigid body.
- `server_address` : indicates the IP address of the MOCAP server. In this example, the server's IP address is' 10.1.1.198 '.

### Launch Default Configuration from Command Line

Run the following command,

```bash
roslaunch mocap_nokov mocap.launch
```

```Then with `rostopic list`, you should be able to see the following topics

```bash
/mocap_node/<tracker_name>/ground_pose
/mocap_node/<tracker_name>/pose
/mocap_node/<tracker_name>/Odom
```
where `<tracker_name>` is usually the name of your tracked objects.
