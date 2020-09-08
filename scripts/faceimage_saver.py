#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
OpenCV FaceImage Saver Node
"""


# 必要なライブラリをインポート
import os
import datetime
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FaceImageSaverNode(object):
    """
    顔画像を保存するクラス
    """

    def __init__(self):
        # サブスクライバを定義
        # 全体画像をサブスクライブするとき、image_callback関数を呼び出す
        rospy.Subscriber("/face_detection/image", Image, self.image_callback)
        # 顔のみの画像をサブスクライブするとき、faceimage_callback関数を呼び出す
        rospy.Subscriber("/face_detection/face_image", Image, self.faceimage_callback)

        # 画像を保持する変数を定義
        self.image = None
        self.faceimage = None

        # 画像の保存先ディレクトリのパスを定義
        self.image_path = os.environ['HOME'] + "/Pictures/"

        # 2秒間処理を待つ
        rospy.sleep(2.0)

    def image_callback(self, input_image):
        """
        受け取った全体画像を保存する関数
        """

        # 全体画像をROSのデータ形式からOpenCV形式に変換して変数に保存
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(input_image, "bgr8")

    def faceimage_callback(self, input_image):
        """
        受け取った顔のみ画像を保存する関数
        """

        # 画像をROSのデータ形式からOpenCV形式に変換して変数に保存
        bridge = CvBridge()
        self.faceimage = bridge.imgmsg_to_cv2(input_image, "bgr8")

    def main(self):
        """
        メインの処理を行う関数
        """

        rospy.loginfo("顔画像を保存するノード")
        rospy.loginfo("[s]キーを押すと顔画像がピクチャーフォルダに保存されます。")
        rospy.loginfo("キーを押すときは、カメラ画像が写っているウィンドウを選択した状態で行ってください。")

        # 一定周期でループを実行するための変数を定義
        rate = rospy.Rate(30)

        # ループを実行
        while not rospy.is_shutdown():
            # 全体画像が保存されているか確認
            if not self.image is None:
                # 画像を表示
                cv2.imshow("FaceImage Saver", self.image)
                # キーボードが押されるまで処理を待つ
                # 1ms待機して入力がなければ次の処理へ進む
                k = cv2.waitKey(1)
                # ESCキーを押したら終了
                if k == 27:
                    break
                # sキーを押したら画像を保存
                elif k == 115:
                    # 画像を指定したディレクトリに保存
                    if not self.faceimage is None:
                        # ファイル名にタイムスタンプを付加して保存
                        timestamp = datetime.datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
                        cv2.imwrite(self.image_path + timestamp + "_face" + ".jpeg", self.faceimage)
                        rospy.loginfo("画像を保存しました。({})".format(timestamp + "_face" + ".jpeg"))
                    else:
                        rospy.loginfo("顔が認識されていません。")

        # 一定時間処理を待つ
        rate.sleep()


if __name__ == '__main__':
    # ノードを宣言
    rospy.init_node("faceimage_saver")
    # クラスのインスタンスを作成し、メイン関数を実行
    FaceImageSaverNode().main()
