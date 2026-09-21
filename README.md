# AtomS3R Foot Angle Tracker

AtomS3R-CAMで、黒い機体上の2つの白マーカーから
**胴体に対する足の相対角度**をリアルタイム計測するファームウェアです。

このリポジトリは
[atoms3r-visual-pose-tracker](https://github.com/temesotejam/atoms3r-visual-pose-tracker)
で確立した白マーカー1D追跡を、足角度センサ用途へ分離したものです。

## Angle definition

- Marker A = 上側レーン
- Marker B = 下側レーン
- 初期直立姿勢 = 0 deg
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
- `cx_px` から `foot_angle_deg` を即時計算
- BMI270は独立200 Hzタスク
- JSON telemetry 10 Hz / 921600 baud
- GitHub Pagesからブラウザ書き込み
- Web SerialでA/B角度をリアルタイム表示

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

    angle_mode = "body_relative_foot_upright_zero_v1"

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
4. 直立でA/Bが約0°付近になることを確認
5. 足を胴体に対して傾ける
6. `foot_angle_deg` が連続的に変化することを確認
7. `detect_fail=0` と `imu.misses=0` を確認
8. 校正範囲内では `angle_in_range=true` であることを確認

## Repository scope

このリポジトリでは足角度推定を発展させます。

元の `atoms3r-visual-pose-tracker` は、
白マーカー位置計測と校正実験の基準リポジトリとして保持します。
