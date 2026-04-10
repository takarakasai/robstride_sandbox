# 制御ブロック構造まとめ

## バイラテラル制御系

### 1. Position Mirroring（位置ミラー）
- **Leader**: MITモード (kp, kd, τ_ff=0)
- **Follower**: MITモード (kp, kd, τ_ff=0)
- **制御則**: Followerの目標位置 = Leaderの現在位置

```
[Leader] --θ,ω--> [Follower]
   ^                |
   |                |
   +----------------+
```

### 2. Force Reflection（力反映）
- **Leader**: MITモード (kp, kd, τ_ff = -force_scale × Followerトルク)
- **Follower**: MITモード (kp, kd, τ_ff=0)
- **制御則**: Followerの目標位置 = Leaderの現在位置、LeaderにFollowerのトルクを反映

```
[Leader] <---τ_f--- [Follower]
   ^        θ,ω        |
   |------------------+
```

### 3. Virtual Coupling（バーチャルカップリング）
- **Leader**: MITモード (kp, kd, τ_ff = -Kc(θ_l-θ_f) - Dc(ω_l-ω_f))
- **Follower**: MITモード (kp, kd, τ_ff = +Kc(θ_l-θ_f) + Dc(ω_l-ω_f))
- **制御則**: LeaderとFollowerを仮想バネ・ダンパで結合

```
[Leader] <==バネ・ダンパ==> [Follower]
```

### 4. Mode Space（モード空間）
- **Leader/Follower**: MITモード (kp, kd, τ_ff=mode変換)
- **制御則**: 両端の合成座標系で制御

### 5. OnDemand（オンデマンド）
- **Leader**: 通常OFF、Followerトルクが閾値超えたらMITモード(τ_ff = -force_scale × τ_f)
- **Follower**: MITモード (kp, kd, τ_ff=0)
- **制御則**: 物体接触時のみLeader ON、力反映

### 6. Position Exchange（古典的位置帰還）
- **Leader**: MITモード (kp, kd, τ_ff=0, 目標=Follower位置)
- **Follower**: MITモード (kp, kd, τ_ff=0, 目標=Leader位置)
- **制御則**: 互いの位置を10kHzで追従


## ユニラテラル制御系

### 1. AssistTest（アシストテスト）
- **Leader**: MITモード (kp, kd, τ_ff=補償項)
- **Follower**: なし
- **制御則**: Leader単体で摩擦・慣性補償等のテスト


---

## 備考
- MITモード: τ = kp(θ_ref-θ) + kd(ω_ref-ω) + τ_ff（10kHz内部制御）
- τ_ff: 外部から与えるフィードフォワードトルク
- 各方式で安全機能（ソフトスタート、トルクリミット、デッドバンド等）有効
