#!/bin/bash
# 重新生成 ego 世界 + 资产断言（照 defop_ws 版移植，2026-10-07 换场建立）。
# 期望值从当前树表 meta 现算 ⇒ 换场/换表自动跟随，不写死常数。
cd /home/ubuntu/zx2026_ego_ws/src/arena_world_gazebo/scripts
echo "=== 重新生成世界 ==="
python3 world_builder.py

echo
echo "=== 校验：残留 static 属性应=0，static 元素应>0 ==="
W=/home/ubuntu/zx2026_ego_ws/src/arena_world_gazebo/worlds/forest_world.world
echo "static 属性残留: $(grep -c 'static="true"' "$W")"
echo "static 元素: $(grep -c '<static>true</static>' "$W")"
echo "模型数: $(grep -c '<model' "$W")"

echo
echo "=== XML 校验 ==="
python3 -c "import xml.dom.minidom as m; m.parse('$W'); print('XML OK')"

echo
echo "=== 场景资产断言（期望值从当前树表 meta 读出，换场自动跟随）==="
EXP=$(python3 - <<'PY'
import os, json, yaml
WS = "/home/ubuntu/zx2026_ego_ws"
cfg = os.path.join(WS, "src/zx2026_common/config")
topo = yaml.safe_load(open(os.path.join(cfg, "scene_topology.yaml")))
tf = [z for z in topo["zones"] if z.get("kind") == "forest"][0]["forest"]["table_file"]
doc = yaml.safe_load(open(os.path.join(cfg, tf)))
meta = doc.get("meta") or {}
trees = doc.get("trees", [])
kind = meta.get("mesh_kind") or "poplar"
mesh = ("meshes/spruce/picea_abies_" if kind == "spruce" else "meshes/poplar/tree_")
other = "meshes/poplar" if kind == "spruce" else "meshes/spruce"
# 每棵树的 mesh 引用数：云杉=1（整树单个 dae）；杨树=2（_wood 干 + _leaves 叶）。
# 沿革：杨树 10-02 晚删冠视觉 ⇒ 曾为 1；10-07 换场恢复叶视觉 ⇒ 回 2。
# 两次都栽在写死常数上（写死 1 在杨树档假 FAIL 406；留 1 在恢复后反向假 FAIL 203）。
mref = 1 if kind == "spruce" else 2
# 冠碰撞：表 meta 明说 crown_collidable 才发；老表无键=True（逐位旧行为）
want_crown = len(trees) if meta.get("crown_collidable", True) else 0
# 真枝碰撞体数 = Σ_变体(该变体每棵枝段数 × 该变体树数)。
# 换场后本表已无 branch_file ⇒ want_branch=0（写死常数会在改模板时假 FAIL，故现算）。
want_branch = 0
bf = meta.get("branch_file")
if bf:
    bp = os.path.join(cfg, bf)
    if os.path.exists(bp):
        tpl = json.load(open(bp))
        cnt = {}
        for t in trees:
            v = int(t["variant"])
            cnt[v] = cnt.get(v, 0) + 1
        for v, n in cnt.items():
            want_branch += len(tpl.get("tree_%d" % v, {}).get("branches", [])) * n
print("%d %s %s %s %d %d %d" %
      (meta.get("n_trees", len(trees)), kind, mesh, other, mref,
       want_crown, want_branch))
PY
)
set -- $EXP
  want_n=$1; kind=$2; mesh=$3; other=$4; mref=$5; want_crown=$6; want_branch=$7
echo "  当前树表: $kind  n=$want_n  mesh/棵=$mref  冠碰撞=$want_crown  枝碰撞体=$want_branch"

RC=0
chk() { # chk <名> <实测> <期望>
  if [ "$2" = "$3" ]; then echo "  OK   $1 = $2"
  else echo "  FAIL $1 = $2（期望 $3）"; RC=1; fi
}
chk "tree_ 模型数"       "$(grep -c '<model name="tree_' "$W")" "$want_n"
chk "$kind mesh 引用"    "$(grep -c "$mesh" "$W")" "$((want_n * mref))"
chk "异种 mesh 残留"     "$(grep -c "$other" "$W")" 0
chk "c_trunk 碰撞体"     "$(grep -c 'c_trunk' "$W")" "$want_n"
chk "c_crown 碰撞体"     "$(grep -c '<collision name="c_crown">' "$W")" "$want_crown"
chk "c_branch 碰撞体"    "$(grep -c '<collision name="c_branch_' "$W")" "$want_branch"
# 叶视觉直证（不靠 mref 推断）：v_leaf 视觉数须=棵数。仅杨树档生效。
if [ "$kind" = "poplar" ]; then
  chk "v_leaf 叶视觉"    "$(grep -c '<visual name="v_leaf">' "$W")" "$want_n"
fi
if [ "$RC" != 0 ]; then
  echo "!! 世界资产不符——检查 scene_topology.table_file 与树表是否配套"
  echo "   （换场回退：table_file 指向旧表 + 恢复 world_builder v_leaf 后重跑本脚本）"
fi
exit $RC
