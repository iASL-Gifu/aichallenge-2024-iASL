import cv2
import numpy as np

# 画像のサイズ
image_width = 600
image_height = 400

# 白い背景の画像を作成
image = np.ones((image_height, image_width, 3), dtype=np.uint8) * 255

# 長方形の座標とサイズ
rect_x, rect_y, rect_width, rect_height = 150, 100, 300, 200

# 楕円の中心と軸の長さ
ellipse_center = (300, 200)
ellipse_major_axis = 150
ellipse_minor_axis = 100
ellipse_angle = 0  # 楕円の角度

# 長方形を描画（青色）
cv2.rectangle(image, (rect_x, rect_y), (rect_x + rect_width, rect_y + rect_height), (255, 0, 0), 2)

# 楕円を描画（緑色）
cv2.ellipse(image, ellipse_center, (ellipse_major_axis, ellipse_minor_axis), ellipse_angle, 0, 360, (0, 255, 0), 2)

# 長方形の中を探索
for y in range(rect_y, rect_y + rect_height + 1):
    for x in range(rect_x, rect_x + rect_width + 1):
        # 楕円の内部にあるかどうかをチェック
        dx = x - ellipse_center[0]
        dy = y - ellipse_center[1]
        if (dx * dx) / (ellipse_major_axis * ellipse_major_axis) + (dy * dy) / (ellipse_minor_axis * ellipse_minor_axis) <= 1.0:
            # 楕円の内部であれば赤色で描画
            image[y, x] = (0, 0, 255)

# 画像の表示
cv2.imshow('Rectangle and Ellipse', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
