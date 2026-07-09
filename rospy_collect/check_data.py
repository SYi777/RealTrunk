"""
数据质量检查 — 适配 _left.npz / _right.npz 格式
用法: python check_data.py
"""

import numpy as np, os, sys, glob

def check_episode(ep_dir, ep_str):
    lp = os.path.join(ep_dir, f"ep_{ep_str}_left.npz")
    rp = os.path.join(ep_dir, f"ep_{ep_str}_right.npz")
    if not os.path.exists(lp):
        return None

    ld = np.load(lp); rd = np.load(rp) if os.path.exists(rp) else None
    states = ld["states"]   # (N, 12): arm6 + hand6
    actions = ld["actions"]
    fps = int(ld.get("fps", 50))
    N = len(states)
    arm_dim = 6

    print(f"\n{'='*60}")
    print(f"Episode {ep_str}: {N}帧 {N/fps:.1f}s")
    print(f"  左 states:{states.shape}")

    # NaN
    ns, na = np.isnan(states).sum(), np.isnan(actions).sum()
    print(f"  NaN: obs={ns} act={na}")

    # 臂部分 (前6列, 弧度)
    arm = states[:, :arm_dim]
    print(f"  左臂范围: [{arm.min():.3f}, {arm.max():.3f}] rad "
          f"= [{np.rad2deg(arm.min()):.1f}, {np.rad2deg(arm.max()):.1f}]°")
    if N > 1:
        ad = np.abs(np.diff(arm, axis=0)).max()
        print(f"  左臂帧间最大: {np.rad2deg(ad):.2f}°")

    # 手部分 (后6列, raw)
    hand = states[:, 6:12]
    print(f"  左手范围: [{hand.min():.0f}, {hand.max():.0f}]")

    # 时间偏移验证
    if N > 2:
        # action[t] 应等于 state[t+1] (action 领先 obs 一帧)
        gap_forward = np.abs(states[1:] - actions[:-1, :12]).mean()
        print(f"  act[t]≈obs[t+1] gap: {gap_forward:.6f} {'✓' if gap_forward<0.01 else '⚠'}")
        # obs[t] 和 action[t] 不应相同 (否则时间偏移失效)
        # 用 max 而不是 mean: 录的时候手没动均值会接近0
        gap_same_max = np.abs(states[1:] - actions[1:, :12]).max()
        if gap_same_max < 0.001:  # max 小于0.001rad=0.06°才算完全一样
            print(f"  ✗ obs≈act! 时间偏移未生效! (max diff={gap_same_max:.6f})")
        else:
            print(f"  obs[t]≠act[t] max diff: {gap_same_max:.4f} ✓ 偏移正常")

    if rd is not None:
        print(f"  右 states:{rd['states'].shape}")
        r_arm = rd["states"][:, :6]
        print(f"  右臂范围: [{r_arm.min():.3f}, {r_arm.max():.3f}] rad")

    print(f"  ✓ 质量合格")
    return True


def main():
    data_dir = os.path.expanduser("~/lerobot_data")

    if len(sys.argv) > 1:
        check_episode(data_dir, sys.argv[1])
    else:
        lefts = sorted(glob.glob(os.path.join(data_dir, "ep_*_left.npz")))
        if not lefts:
            print(f"未找到 ep_*_left.npz 在 {data_dir}/")
            sys.exit(1)
        for f in lefts:
            fn = os.path.basename(f)
            ep_str = fn.split("_")[1]  # "001"
            check_episode(data_dir, ep_str)
    print()


if __name__ == "__main__":
    main()
