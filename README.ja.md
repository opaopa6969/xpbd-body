[English](./README.md) · **日本語**

# xpbd-body

> VRM上半身のための、ゼロから書いた小さな **XPBD 能動ラグドール** ―― 本物の重力・質量・運動量・接触を持ちながら **目標ポーズを追従する**。

[motion-engine](https://github.com/opaopa6969/motion-engine) の L3(動力学)層: motion-engine が運動学的に目標ポーズを*生成*する(L2)のに対し、`xpbd-body` は**実際の質量と重力を持つカラダ**に、コンプライアンス(柔らかさ)を持った「筋肉」モーターを通じてそれを**物理的に追従**させる ―― だから弱ければ垂れ下がり、突かれれば反応し、接触を尊重する。**pure・依存ゼロ・決定論的**(固定サブステップ、`Math.random` 不使用)なので、ヘッドレスで動作し Node 上で単体テストされ、決定論的リプレイとの互換性を保つ。

```js
import { World, makeArm, q4 } from 'xpbd-body';

const world = new World();
const arm = makeArm(world, { mass: 1.2, compliance: 0.0006 });   // shoulder→upper→lower

// each frame: feed the L2 pose as the motor targets, step, read the result
arm.setTarget(qUpper, qLower);     // child-in-parent quaternions (e.g. from motion-engine)
world.step(1 / 60);
const handWorld = arm.handPos();   // arm.upper.q / arm.lower.q drive the VRM bones
```

## なぜ XPBD か

位置ベース物理では、関節の**アタッチメント**、角度の**モーター**(「筋肉」)、関節可動域、そして**接触**が、すべて*同種*の制約であり、サブステップングによって硬いモーターの下でも安定する。この一様性こそが、次に来るもの ―― 自己衝突(「腕がお腹を回り込む」)が単なる別の接触制約として素直に組み込める理由 ―― に対する正しい基盤である。

全体はたった4つのプリミティブでできている: **`World` / `Body` / `Attach`(関節) / `Motor`(筋肉)**。

## API

- `new World({ gravity?, linDamp?, angDamp? })` → `step(dt, substeps=20)`、`add(body)`、`constrain(c)`。
- `new Body({ pos, q, mass, radius, cr?, fixed })` ―― 剛体(今のところ等方慣性)。`cr` = M3 の接触判定用衝突半径(既定は `radius`)。
- `Attach(A, rA, B, rB, compliance=0)` ―― ローカル点を一致させ続けるボールジョイント(チェーンを保つために繰り返し解かれる)。
- `Motor(A, B, restQuat, compliance)` ―― B の向きを A に対して相対的に `restQuat` へ駆動する。compliance = 筋肉の柔らかさ(小=強い、大=垂れる)。
- `GroundContact(B, { y, compliance })` / `BoxContact(B, min, max, { compliance })` / `Contact(A, B, { compliance })` ―― 片側の接触制約(M3): 物体を平面の上に保つ(机の天板)、AABB の外に保つ(牌・机の縁)、または他の物体から離す(自己衝突)。めり込むまでは何もせず(no-op)、めり込んだら押し出す。関節と一緒に繰り返し解かれる。
- `makeArm(world, opts)` → `{ upper, lower, setTarget(qU,qL), handPos() }` ―― 2ボーンの能動ラグドール腕。
- `makeUpperBody(world, { ..., profile })` → `{ bodies, motors, setPose(poseEuler) }` ―― 骨盤にアンカーされた上半身チェーン(M2)。`setPose` は motion-engine のポーズ(`{bone:[x,y,z]}`)をモーターの目標として受け取る。物理的な結果は `bodies[name].q` から読む。`profile`(BodyProfile: `{ mass, bulk, selfCollision }`、`DEFAULT_PROFILE` 参照)がカラダに体格を与える ―― `mass` は垂れ具合を、`bulk` は胴体の太さを、`selfCollision` は前腕/手を胴体の周りに回り込ませる(貫通させない)。省略すればボディは M2 のボディとバイト単位で一致する。`qFromEulerXYZ`、`UPPER_BODY` もエクスポートされている。

## 逆動力学 (M4) ―― `xpbd-body/inverse`

能動ラグドールは既に**前向きモデル**(制御→軌道)だ。M4 はそれを*逆向き*に回す機構を足す: 観測された姿勢軌道から、それを生んだ**制御**を推定する。[keiko-engine](https://github.com/opaopa6969/keiko-engine) が「動きの記述子＝生の2D位置ではなく、推定した制御」を採るために必要なもの。詳細は **[docs/inverse-dynamics.ja.md](./docs/inverse-dynamics.ja.md)**。

```js
import { makeRig, simulate, estimateControl } from 'xpbd-body/inverse';

const rig = makeRig({ compliance: 0.00006 });        // カラダ + prior(可動域・トルク上限)
const traj = simulate(rig, control, 1 / 60, 120);    // 制御→軌道。pure・決定論
const { control, residual, feasible } = estimateControl(rig, observedTrajectory);
```

- `snapshot(world)` / `restore(world, snap)` ―― 全 body と**モーターの制御状態**を plain data でダンプ/復元。推定ループが巻き戻して、ビット単位で同一の初期状態から別の制御を試せる。
- `simulate(rig, controlTrajectory, dt, steps)` → `poseTrajectory` ―― 前向きモデル。**副作用なし**(world を snapshot して restore する)・**決定論**。逐次の `world.step(dt)` API はそのまま。
- `estimateControl(rig, observed, opts)` → `{ control, residual, feasible, violations, unique }` ―― analysis by synthesis: 制御を仮定し、前向きに回し、ズレを見て、仮定を直す。勾配なし(座標降下 / seed付きCEM)、`Math.random` は不使用。`feasible: false` は、**可動域**を破った関節・**トルク上限**を超えた筋肉・あるいは「**このカラダにはできない**」と言う**残差**を名指しする。

**この逆問題は不良設定(ill-posed)**であり、API はそれを明言する(`unique: false`): 同じ見た目の動きを生む制御は複数あり、接触力は観測できず、単眼の観測に奥行きは無い。`estimateControl` が返すのは「*その*制御」ではなく、残差と平滑化正則化が選んだ「*1つの*制御」だ。[限界はドキュメントに書いてある。](./docs/inverse-dynamics.ja.md)

## テスト

```sh
node test.mjs     # or: npm test
npm run mcp:test  # MCP サーバの e2e（起動 → tools/resources を叩く）
```

能動ラグドールのヘッドレスな証明: 硬いモーターの下でも安定、関節が繋がったまま、強い筋肉が目標を追従、弱い/重い腕は重力で垂れ、突かれると乱れてから回復し、決定論的である。加えて M4 の逆動力学層: **合成往復検証**(既知の制御 → 軌道 → 推定 → 元の制御に rms 0.003 rad 以内で戻る)、巻き戻して別の制御を試す、そして3つの infeasible 判定すべて。

## MCP

このライブラリは **MCP サーバ**でもある（namespace `xpbd`・[volta](https://github.com/opaopa6969/volta-mcp) 参加）。前向きモデルと逆動力学層を tool として公開し、他の MCP サービスが組み合わせられるようにする。

- **仕様**: `xpbd://spec`（機械可読）。**ガイド**: `xpbd://guide`。
- **tools**: `simulate`（ポーズ→物理追従）、`estimate_control_start/status/result`（観測→制御推定・job 型）、`check_feasible`（物理可能性判定）。
- **ローカル起動**: `PORT=9204 npm run mcp:start` → `curl http://127.0.0.1:9204/healthz`。
- **設計**: `docs/mcp/DESIGN.md`。**状況**: `docs/mcp/STATUS.md`。**skill**: `docs/skills/xpbd-body-mcp-usage/SKILL.md`。

## ステータス

**M1** XPBD コア + 2ボーンの能動ラグドール腕。**M2** motion-engine のポーズで駆動する、骨盤アンカー付きの上半身チェーン全体(`makeUpperBody` + `setPose`)。**M3** 片側 XPBD 制約としての接触 ―― 地面/平面(机)、AABB ボックス(牌/縁)、`BodyProfile` に駆動される球↔球の自己衝突(`bulk` が胴体を太らせ、肢体がその周りを回り込む)。**M4**(本リリース)逆動力学 ―― snapshot/restore、pure な前向きモデル、勾配なしの制御推定。ロードマップ: 接触摩擦 ・ 事後判定ではなく XPBD 制約としての関節可動域 ・ keiko-engine 統合。

## ライセンス

MIT
