# AtomS3R Foot Angle Tracker

AtomS3R-CAMで、黒い機体上の2つの白マーカーから
**胴体に対する足の相対角度**をリアルタイム計測するファームウェアです。

このリポジトリは
[atoms3r-visual-pose-tracker](https://github.com/temesotejam/atoms3r-visual-pose-tracker)
で確立した白マーカー1D追跡を、足角度センサ用途へ分離したものです。

## Angle definition

- Marker A = 上側レーン
- Marker B = 下側レーン
- 起動時にIMUで確認した直立・静止姿勢 = 0 deg
- 正方向 = 白マーカーのX位置が小さくなる方向
- 出力 = 足・脚剛体リンクの**胴体に対する相対角度**

校正時は足を世界に固定して胴体を傾けました。
本番では胴体に対して足が動きますが、カメラは胴体に剛固定、
白マーカーは足に剛固定されているため、カメラが観測する相対姿勢は同じです。

## Calibration v1

2026-09-21の校正ログから、初期静止区間 frame 57..97 を
直立0°として使用しています。

A:

    x0 = 169.615317 px
    theta_A = 0.167779119 * (169.615317 - x_A)

B:

    x0 = 174.843512 px
    theta_B = 0.162645305 * (174.843512 - x_B)

詳細は:

- `calibration/foot_angle_v1.json`
- `docs/foot-angle-calibration-v1.md`

を参照してください。

## Current runtime

- GC0308 / QVGA 320x240 / grayscale
- 白ベタマーカー専用1D検出
- A/Bとも前フレーム不要
- ArUcoなし
- template matchingなし
- pyramidなし
- Otsuなし
- IMU重力方向＋安定性から起動時0°を自動調整
- 自動調整後、`cx_px` から `foot_angle_deg` を即時計算
- BMI270は独立200 Hzタスク
- JSON telemetry 10 Hz / 921600 baud
- GitHub Pagesからブラウザ書き込み
- Web SerialでA/B角度をリアルタイム表示

## Automatic upright zero v2

起動時の0°は固定X値ではなく、実機のその起動時の直立姿勢から決めます。

直立判定は**IMUのみ**で行い、白マーカー位置は直立判定には使いません。
現在の取り付けでは直立時の重力方向がIMU -Z方向に対応します。

以下を連続2秒満たした場合だけゼロ平均を進めます。

- 重力方向がIMU -Zから5°以内
- 加速度ノルムが1 g ±0.03 g
- 3軸合成角速度が1.5 deg/s以下
- A/B両方の白マーカーが有効

途中でIMU条件またはマーカー検出が崩れた場合、それまでの平均は破棄して最初からやり直します。
条件成立中に最低15画像サンプルを平均し、

    x_zero,A = mean(cx_A)
    x_zero,B = mean(cx_B)

として、その起動中は再ゼロ化しません。

角度は

    theta_A = 0.167779119 * (x_zero,A - x_A)
    theta_B = 0.162645305 * (x_zero,B - x_B)

で求めます。

自動ゼロ確定前も生の `cx_px` と暫定角度値はログへ出しますが、
`angle_valid=false` とするため、制御やWeb UIでは有効角度として扱いません。

> IMUが判断できるのは「胴体が直立していること」です。
> そのため、起動時には足も意図した0°姿勢にしておくことが前提です。

## Telemetry

各マーカーには以下を出します。

    valid
    source
    cx_px
    cy_px
    foot_angle_deg
    angle_valid
    angle_in_range
    peak_contrast
    weight_sum
    bright_width_px
    detect_ok
    detect_fail
    vision_us

`angle_in_range=false` の場合も角度は計算しますが、
校正で実測した範囲外への外挿です。

トップレベルには

    angle_mode = "body_relative_foot_auto_zero_v2"

を出します。

IMUログも残しているため、胴体絶対姿勢との組み合わせや
今後の検証にも利用できます。

## Build

    python -m pip install platformio
    pio run -e atoms3r_cam

## Hardware test

1. 白マーカーを現在のA/B位置に取り付ける
2. Pagesから最新版を書き込む
3. 921600 baudでシリアル接続
4. 足も0°姿勢にして機体を直立・静止させる
5. Web UIまたはログで `zeroing.ready=true` になるまで保持する
6. 自動ゼロ後にA/Bが0°付近になることを確認
7. 足を胴体に対して傾ける
8. `foot_angle_deg` が連続的に変化することを確認
9. `detect_fail=0` と `imu.misses=0` を確認
10. 校正範囲内では `angle_in_range=true` であることを確認

## Repository scope

このリポジトリでは足角度推定を発展させます。

元の `atoms3r-visual-pose-tracker` は、
白マーカー位置計測と校正実験の基準リポジトリとして保持します。
