このプログラムはヒューマンアカデミー株式会社のAI入門講座で使用するプログラムです。

# OpenCV FaceImage Saver
顔認識をして顔部分だけクロップされた画像を保存するパッケージ

## Requirements
* [uvc_camera](http://wiki.ros.org/uvc_camera)
* [opencv_apps](http://wiki.ros.org/opencv_apps)

## Installation

```
cd ~/catkin_ws
rosdep install -r -y -i --from-paths src
```

## Usage
```sh
roslaunch opencv_faceimage_saver opencv_faceimage_saver.launch
```

- 起動後、顔認識の結果が表示されます。
- `s`キーを押すと顔だけを切り取った画像が`Pictures`フォルダに`タイムスタンプ_face.jpeg`という名前で保存されます。
