# ヤコビ行列を用いた逆運動学ソルバーの解説

このドキュメントでは、`k`ライブラリにおける逆運動学（Inverse Kinematics, IK）ソルバーの実装を解説します。

---

## 1. 概要

本ライブラリの逆運動学ソルバーは**ヤコビアン法（Jacobian-based method）**を採用しています。これは反復的な数値解法であり、以下の流れで解を求めます：

```
現在の関節角度 → ヤコビ行列計算 → 逆行列で角速度計算 → 関節角度更新 → 収束まで反復
```

## 2. ヤコビ行列とは

ヤコビ行列（Jacobian Matrix）は、関節速度とエンドエフェクタの速度を結びつける行列です：

$$
\dot{x} = J(q) \cdot \dot{q}
$$

- $\dot{x}$: エンドエフェクタの速度（6次元：線速度3 + 角速度3）
- $\dot{q}$: 関節速度（n次元：関節数）
- $J(q)$: ヤコビ行列（6×n）

## 3. ヤコビ行列の計算方法

### 3.1 解析的ヤコビアン vs 数値的ヤコビアン

| 手法 | 説明 | 特徴 |
|------|------|------|
| **解析的ヤコビアン** | 幾何学的関係から直接計算 | 高精度、計算効率が良い |
| **数値的ヤコビアン** | 有限差分法で近似 | 実装が簡単だが精度・効率で劣る |

本ライブラリは**解析的ヤコビアン**を使用しています。

### 3.2 実装詳細

ヤコビ行列の各列は、対応する関節がエンドエフェクタの速度に与える寄与を表します。

**回転関節（Rotational Joint）の場合：**

```rust
// src/funcs.rs:24-32
let a_i = t_i.rotation * axis;  // ジョイント軸（世界座標系）
let dp_i = a_i.cross(&(p_n.clone().vector - p_i.vector));
// 線速度: a_i × (p_end - p_joint)
// 角速度: a_i
```

数学的には：
$$
J_i = \begin{bmatrix} a_i \times (p_{end} - p_i) \\ a_i \end{bmatrix}
$$

**直動関節（Linear Joint）の場合：**

```rust
// src/funcs.rs:17-22
let p_i = t_i.rotation * axis;
// 線速度: 軸方向ベクトル
// 角速度: ゼロ
```

数学的には：
$$
J_i = \begin{bmatrix} a_i \\ 0 \end{bmatrix}
$$

## 4. 逆運動学の解法

### 4.1 基本アルゴリズム

逆運動学では、目標姿勢 $x_{target}$ に到達するための関節角度 $q$ を求めます。

反復法の各ステップ：

1. 現在姿勢と目標姿勢の誤差を計算: $e = x_{target} - x_{current}$
2. ヤコビ行列を計算: $J(q)$
3. 関節角度の更新量を計算: $\Delta q = J^{-1} \cdot e$
4. 関節角度を更新: $q_{new} = q_{old} + \alpha \cdot \Delta q$
5. 収束判定（誤差が許容値以下なら終了）

$\alpha$ は学習率（本ライブラリでは `jacobian_multiplier`、デフォルト0.5）

### 4.2 逆行列計算の手法

本ライブラリは状況に応じて3種類の逆行列計算法を使い分けています：

#### a) LU分解（完全系）

```rust
jacobi.lu().solve(&err)
```

- **条件**: 自由度が必要十分（例：6DoFアームで6自由度の目標）
- **特徴**: 正方行列の場合に最も効率的
- **計算量**: O(n³)

#### b) SVD法（冗長系）

```rust
jacobi.svd(true, true).solve(&err, EPS)
```

- **条件**: 自由度が過剰（例：7DoFアームで6自由度の目標）
- **特徴**: 最小ノルム解を求める
- **計算量**: O(mn²) （m×n行列の場合）

#### c) 疑似逆行列 + ナルスペース投影（冗長系 + サブタスク）

```rust
let jacobi_inv = jacobi.pseudo_inverse(EPS).unwrap();
let d_q = jacobi_inv * err + (I - jacobi_inv * jacobi) * subtask;
```

- **条件**: 冗長系かつナルスペース関数が指定されている
- **特徴**: メインタスクとサブタスクを同時に最適化

### 4.3 手法選択のロジック

```rust
// src/ik.rs より簡略化
if available_dof > required_dof {
    // 冗長系
    match self.nullspace_function {
        Some(_) => /* 疑似逆行列 + ナルスペース投影 */,
        None    => /* SVD法 */,
    }
} else {
    // 完全系 → LU分解
}
```

## 5. 冗長系とナルスペース

### 5.1 冗長系とは

エンドエフェクタの目標（通常6自由度：位置3 + 姿勢3）に対して、ロボットの関節数が多い場合を「冗長系」と呼びます。例えば7自由度のロボットアームは1自由度の冗長性を持ちます。

### 5.2 ナルスペース（Null Space）

ナルスペースとは、ヤコビ行列の零空間であり、エンドエフェクタの位置・姿勢を変えずに関節を動かせる方向を表します。

$$
\Delta q_{null} \in \text{Null}(J) \Rightarrow J \cdot \Delta q_{null} = 0
$$

### 5.3 複合目的関数

冗長系では、メインタスク（エンドエフェクタ制御）とサブタスク（追加目標）を同時に達成できます：

$$
\Delta q = J^+ e + (I - J^+ J) \nabla H
$$

- 第1項: メインタスク（目標姿勢への収束）
- 第2項: サブタスク（ナルスペース内での最適化）

本ライブラリでは、サブタスクとして「基準姿勢への復帰」が実装されています：

```rust
// src/ik.rs:489-511
// コスト関数: H(q) = 1/2(q - q_ref)ᵀ W (q - q_ref)
// 勾配: ∇H = W(q - q_ref)
derivative_vec[i] = weight[i] * (positions[i] - reference[i]);
```

## 6. 収束判定

位置誤差と回転誤差を別々に評価します：

```rust
// src/ik.rs:329-345
if len_diff.norm() < self.allowable_target_distance   // デフォルト: 0.001m
    && rot_diff.norm() < self.allowable_target_angle  // デフォルト: 0.005rad
{
    return Ok(());  // 収束成功
}
```

最大反復回数（デフォルト10回）に達しても収束しない場合は `NotConvergedError` を返します。

## 7. 制約機能

### 7.1 操作空間の制約

6自由度すべてを制御する必要がない場合、一部を無視できます：

```rust
let constraints = Constraints {
    position_x: true,   // X位置を制御
    position_y: true,   // Y位置を制御
    position_z: true,   // Z位置を制御
    rotation_x: false,  // X軸回転は無視
    rotation_y: true,   // Y軸回転を制御
    rotation_z: true,   // Z軸回転を制御
    ..Default::default()
};
```

### 7.2 関節の固定

特定の関節を動かさないように指定できます：

```rust
constraints.ignored_joint_names = vec!["wrist_roll".to_string()];
```

## 8. ソルバーのパラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `allowable_target_distance` | 0.001 | 位置誤差の許容値（メートル） |
| `allowable_target_angle` | 0.005 | 回転誤差の許容値（ラジアン） |
| `jacobian_multiplier` | 0.5 | 学習率（更新量の倍率） |
| `num_max_try` | 10 | 最大反復回数 |

## 9. 使用例

```rust
use k::{JacobianIkSolver, Chain, SerialChain};

// ロボットモデルの読み込み
let robot = Chain::<f64>::from_urdf_file("robot.urdf").unwrap();
let arm = robot.find_serial_chain_from_end("end_effector").unwrap();

// IKソルバーの作成
let solver = JacobianIkSolver::new(
    0.001,  // allowable_target_distance
    0.001,  // allowable_target_angle
    0.8,    // jacobian_multiplier
    100,    // num_max_try
);

// 目標姿勢
let target_pose = Isometry3::new(
    Vector3::new(0.5, 0.2, 0.3),  // 位置
    Vector3::new(0.0, 0.0, 0.0),  // 回転（軸角度表現）
);

// IK解法実行
solver.solve(&arm, &target_pose).unwrap();

// 結果の取得
let joint_angles = arm.joint_positions();
```

## 10. 他のIK手法との比較

| 手法 | 特徴 | 用途 |
|------|------|------|
| **ヤコビアン法**（本ライブラリ） | 数学的に厳密、特異点の扱いが明確 | 産業用ロボット、精密制御 |
| **CCD法** | 実装が簡単、高速 | ゲーム、アニメーション |
| **FABRIK法** | 直感的、制約処理が容易 | キャラクターアニメーション |
| **解析解** | 最速、最も正確 | 特定構造のロボットのみ |

## 11. 関連ファイル

| ファイル | 内容 |
|---------|------|
| `src/ik.rs` | IKソルバーのメイン実装 |
| `src/funcs.rs` | ヤコビ行列計算 |
| `src/chain.rs` | キネマティックチェーン定義 |
| `src/errors.rs` | エラー型定義 |
| `tests/test_ik.rs` | IKソルバーのテスト |
| `examples/interactive_ik.rs` | インタラクティブなIKデモ |

## 12. 参考文献

- 梶田秀司『ヒューマノイドロボット』オーム社
- Bruno Siciliano et al., "Robotics: Modelling, Planning and Control", Springer
- Richard M. Murray et al., "A Mathematical Introduction to Robotic Manipulation"
