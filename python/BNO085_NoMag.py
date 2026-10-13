import time
import serial
from vpython import (
    canvas,
    box,
    arrow,
    vector,
    color,
    rate
)
from scipy.spatial.transform import Rotation


COM_PORT = "COM3"
BAUD_RATE = 115200

BODY_LENGTH = 4.0
BODY_HEIGHT = 1.0
BODY_WIDTH = 2.0


def imu_to_vpython(v):
    """
    IMU座標からVPython座標への変換。

    IMU +X → VPython +Z
    IMU +Y → VPython +Y
    IMU +Z → VPython -X

    Zだけ-Xへ変換することで右手座標系を維持する。
    """
    return vector(
        v[0],
        v[1],
        v[2]
    )


ser = serial.Serial(
    COM_PORT,
    BAUD_RATE,
    timeout=1
)

# COMポートを開いたときのマイコン再起動を待つ
time.sleep(2)
ser.reset_input_buffer()


scene = canvas(
    title=(
        "BNO085 Game Rotation Vector\n"
        "赤=X軸、緑=Y軸、青=Z軸、Rキー=基準姿勢を再設定"
    ),
    width=900,
    height=600,
    background=color.gray(0.2)
)

scene.autoscale = False
scene.range = 5

# 斜めから見ることで直方体の3次元形状を確認しやすくする
scene.forward = vector(-1, -0.4, -1)


body = box(
    pos=vector(0, 0, 0),
    length=BODY_LENGTH,
    height=BODY_HEIGHT,
    width=BODY_WIDTH,
    color=color.cyan
)

# IMU本体の各軸を表す矢印
x_arrow = arrow(
    pos=vector(0, 0, 0),
    color=color.red,
    shaftwidth=0.07
)

y_arrow = arrow(
    pos=vector(0, 0, 0),
    color=color.green,
    shaftwidth=0.07
)

z_arrow = arrow(
    pos=vector(0, 0, 0),
    color=color.blue,
    shaftwidth=0.07
)


reference_rotation = None
reset_requested = False


def key_down(event):
    global reset_requested

    if event.key.lower() == "r":
        reset_requested = True


scene.bind("keydown", key_down)

print("姿勢データを待っています。")
print("IMUを基準にしたい向きで静止させてください。")


try:
    while True:
        rate(60)

        line = ser.readline().decode(
            "utf-8",
            errors="ignore"
        ).strip()

        print("受信:", repr(line)) 

        if not line:
            continue

        values = line.split(",")

        # Arduinoの新形式：Q,w,x,y,z
        if len(values) == 5 and values[0] == "Q":
            quaternion_values = values[1:]

        # Arduinoの旧形式：w,x,y,z
        elif len(values) == 4:
            quaternion_values = values

        else:
            print("形式が違います:", line)
            continue

        try:
            w, x, y, z = map(float, quaternion_values)
        except ValueError:
            print("数値に変換できません:", line)
            continue

        # クォータニオンが変化しているか確認
        print(f"w={w:.3f}, x={x:.3f}, y={y:.3f}, z={z:.3f}")

        try:
            current_rotation = Rotation.from_quat([x, y, z, w])
        except ValueError:
            print("クォータニオンが不正です:", line)
            continue

        # 最初の有効な姿勢を基準姿勢にする
        if reference_rotation is None or reset_requested:
            reference_rotation = current_rotation
            reset_requested = False
            print("基準姿勢を設定しました。")

        # 起動時の姿勢からの相対回転
        relative_rotation = (
            reference_rotation.inv()
            * current_rotation
        )

        # IMU本体のX・Y・Z軸が現在どこを向いているか
        imu_x = relative_rotation.apply([1, 0, 0])
        imu_y = relative_rotation.apply([0, 1, 0])
        imu_z = relative_rotation.apply([0, 0, 1])

        # VPython座標へ変換
        vp_x = imu_to_vpython(imu_x)
        vp_y = imu_to_vpython(imu_y)
        vp_z = imu_to_vpython(imu_z)

        # 直方体の長手方向をIMUのX軸に合わせる
        body.axis = BODY_LENGTH * vp_x

        # 直方体の上方向をIMUのZ軸に合わせる
        body.up = vp_z

        # IMUの軸を表示
        x_arrow.axis = 2.5 * vp_x
        y_arrow.axis = 2.5 * vp_y
        z_arrow.axis = 2.5 * vp_z

except KeyboardInterrupt:
    print("終了します。")

finally:
    ser.close()