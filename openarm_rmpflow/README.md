# OpenARM × Isaac Sim 6.0 — RmpFlow 運動生成

在 **Isaac Sim 6.0** 裡讓 **OpenARM 雙臂**用 **RmpFlow（Lula）** 做「給目標座標 → 手臂移動過去」的
點對點運動生成（point-to-point motion generation）。純模擬。

**狀態（2026-07-20）**：✅ 左臂點對點運動成功、收斂並 hold；✅ 左臂 36 顆碰撞球已由 `gen_collision_spheres.py` 產出並寫入 lula description（覆蓋率~100%、frame 驗證過）。

## 檔案

```
openarm_rmpflow/
├── openarm_rmpflow_run.py          # 控制腳本（貼進 Isaac Sim Script Editor 執行）
├── gen_collision_spheres.py        # 讀 URDF collision STL → PCA 主軸切段產碰撞球，寫回 lula yaml
├── verify_spheres.py               # 幾何自檢：各 link 覆蓋率 / 最大外露
└── config/
    ├── openarm_left_lula_description.yaml    # 左臂骨架 + 36 顆碰撞球（cspace / limits / cspace_to_urdf_rules / collision_spheres）
    ├── openarm_left_rmpflow_config.yaml      # 左臂 RMP 參數（由官方 FR3 版衍生）
    ├── openarm_right_lula_description.yaml   # 右臂（已產生，尚未測載入）
    └── openarm_right_rmpflow_config.yaml
```

## 怎麼跑

1. Isaac Sim 6.0 GUI 開好（容器內 `/opt/isaacsim/bin/isaacsim`）。
2. `Window > Script Editor`，把 `openarm_rmpflow_run.py` 貼進去執行。
   - 腳本裡的路徑指向容器內 `openarm_description`；換環境請改開頭幾個路徑常數與 `TARGET_POS`。
3. 執行後印出 `[ready]` → **手動按工具列 ▶ Play**，左臂即朝目標點移動並收斂。
4. 支援 ■ Stop → ▶ Play 自動重新初始化（不會軟掉）。

## ⚠️ 已知限制 / 待補

- 左臂 `collision_spheres`（36 顆）已產出並寫入，**幾何驗證過但 Lula 實際 parse 尚未 headless 驗**（留 GUI 載入時一併看）；避障要看效果需 `rmpflow.add_obstacle(...)` 註冊障礙物。
- 右臂碰撞球**尚未產**（`gen_collision_spheres.py right` 同法可補）。
- `body_cylinders` 是泛用近似，非照 OpenARM 實際尺寸。
- 右臂兩份 yaml 已產生但**尚未測載入**。
- 這是「能動」的起點，不是「調校完成」的成品。

## 為什麼不用 `World`（重要）

本環境用 `World.reset()` / `reset_async()` 初始化物理**必失敗**
（`RuntimeError: Physics context is not initialized`；Isaac Sim 6.0 的 `SimulationManager`
不把 stage 上的 `/PhysicsScene` 註冊成 default physics scene）。
因此腳本改走：**不建 World** → 手動按 ▶ Play 讓 GUI 原生起物理 → 掛 `omni.physx` step callback 驅動。
另有兩個必備前置：轉檔產物需補 PhysX articulation 旗標、關節 drive 需 `set_gains`（否則 gain=0 手臂亂甩）。

## 出處 / 授權

- 機器人本體 `openarm_description` 為 **Enactic, Inc.** 維護。
- `*_rmpflow_config.yaml` 衍生自 **NVIDIA Isaac Sim 官方 FR3 RmpFlow config**。
- 本資料夾僅為個人學習/實驗紀錄。
