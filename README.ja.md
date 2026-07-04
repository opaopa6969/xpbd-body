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
- `makeUpperBody(world, { ..., profile })` → `{ bodies, setPose(poseEuler) }` ―― 骨盤にアンカーされた上半身チェーン(M2)。`setPose` は motion-engine のポーズ(`{bone:[x,y,z]}`)をモーターの目標として受け取る。物理的な結果は `bodies[name].q` から読む。`profile`(BodyProfile: `{ mass, bulk, selfCollision }`、`DEFAULT_PROFILE` 参照)がカラダに体格を与える ―― `mass` は垂れ具合を、`bulk` は胴体の太さを、`selfCollision` は前腕/手を胴体の周りに回り込ませる(貫通させない)。省略すればボディは M2 のボディとバイト単位で一致する。`qFromEulerXYZ`、`UPPER_BODY` もエクスポートされている。

## テスト

```sh
node test.mjs     # or: npm test
```

能動ラグドールのヘッドレスな証明: 硬いモーターの下でも安定、関節が繋がったまま、強い筋肉が目標を追従、弱い/重い腕は重力で垂れ、突かれると乱れてから回復し、決定論的である。

## ステータス

**M1** XPBD コア + 2ボーンの能動ラグドール腕。**M2** motion-engine のポーズで駆動する、骨盤アンカー付きの上半身チェーン全体(`makeUpperBody` + `setPose`)。**M3**(本リリース)片側 XPBD 制約としての接触 ―― 地面/平面(机)、AABB ボックス(牌/縁)、`BodyProfile` に駆動される球↔球の自己衝突(`bulk` が胴体を太らせ、肢体がその周りを回り込む)。ロードマップ: M4 ホストへオプトインの物理モードとして統合(netmahg の `?phys=1` で実現済み) ・ 接触摩擦 ・ プロファイルからの関節可動域。

## ライセンス

MIT
