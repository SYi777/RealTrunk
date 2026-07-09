#!/usr/bin/env python3
"""
单步推理调试 — 按 Enter 预测, 再按 Enter 执行
=============================================
  [Enter] 读关节 → 打印归一化值+反归一化值
  [Enter] 发操控 (臂 JointPos + 手 TCP)
  [s]   只读不预测
  [q]   退出

用法: source /opt/ros/noetic/setup.bash
      python3 step_inference.py --model policy_best.pt --norm norm_params.pt
"""

import rospy, sys, time, socket, torch, numpy as np
from sensor_msgs.msg import JointState
from embodied_arm_msgs.msg import JointPos

try: from embodied_arm_msgs.msg import Hand_Status; HAS_HAND = True
except: HAS_HAND = False

# ── 复刻 deploy_test 的接口 ──
LEFT_IP = "169.254.128.18"; RIGHT_IP = "169.254.128.19"; ARM_PORT = 8080
CMD_MODBUS  = b'{"command":"close_modbus_mode","port":1}\r\n'
CMD_RM_PLUS = b'{"command":"set_rm_plus_mode","mode":0}\r\n'


class StepInference:
    def __init__(self, model_path, norm_path, side="both",
                 state_dim=24, action_dim=24,
                 hidden_dim=256, dim_feedforward=1024, chunk_size=50):
        self.side = side
        # ── 模型 ──
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"设备: {self.device}")

        # ACTPolicy 定义内联，不依赖 train.py
        from torch import nn as _nn
        class ACTPolicy(_nn.Module):
            def __init__(self, state_dim, action_dim, hidden_dim=512,
                         dim_feedforward=3200, chunk_size=50,
                         n_encoder_layers=4, n_decoder_layers=7, kl_weight=1.0):
                super().__init__()
                self.chunk_size = chunk_size; self.kl_weight = kl_weight
                self.input_proj = _nn.Linear(state_dim, hidden_dim)
                self.action_proj = _nn.Linear(action_dim, hidden_dim)  # train.py 存了但没用
                self.query_embed = _nn.Embedding(chunk_size, hidden_dim)
                el = _nn.TransformerEncoderLayer(d_model=hidden_dim, nhead=8,
                     dim_feedforward=dim_feedforward, batch_first=True)
                self.encoder = _nn.TransformerEncoder(el, num_layers=n_encoder_layers)
                self.latent_proj = _nn.Linear(hidden_dim, hidden_dim * 2)
                dl = _nn.TransformerDecoderLayer(d_model=hidden_dim, nhead=8,
                     dim_feedforward=dim_feedforward, batch_first=True)
                self.decoder = _nn.TransformerDecoder(dl, num_layers=n_decoder_layers)
                self.output_proj = _nn.Linear(hidden_dim, action_dim)
            def forward(self, state, action=None):
                B = state.shape[0]
                s = self.input_proj(state).unsqueeze(1)
                m = self.encoder(s)
                p = self.latent_proj(m[:,0]); mu, lv = p.chunk(2,dim=-1)
                if self.training and action is not None:
                    std = torch.exp(0.5*lv); eps = torch.randn_like(std)
                    kl = -0.5*(1+lv-mu.pow(2)-lv.exp()).mean()
                else: kl = torch.tensor(0.0)
                q = self.query_embed(torch.arange(self.chunk_size,device=state.device))
                q = q.unsqueeze(0).expand(B,-1,-1)
                out = self.decoder(q, m)
                return self.output_proj(out), kl
            def predict(self, state):
                self.eval()
                with torch.no_grad(): a,_ = self.forward(state,None); return a.squeeze(0)

        norm = torch.load(norm_path, map_location=self.device)
        self.s_mean = norm["s_mean"].to(self.device)
        self.s_std  = norm["s_std"].to(self.device)
        self.a_mean = norm["a_mean"].to(self.device)
        self.a_std  = norm["a_std"].to(self.device)

        self.policy = ACTPolicy(state_dim=state_dim, action_dim=action_dim,
                                hidden_dim=hidden_dim,
                                dim_feedforward=dim_feedforward,
                                chunk_size=chunk_size).to(self.device)
        self.policy.load_state_dict(torch.load(model_path, map_location=self.device))
        self.policy.eval()
        print(f"模型: {model_path}")

        # ── ROS 订阅 (同 deploy_test) ──
        self.l_pos = [0.0]*6; self.r_pos = [0.0]*6
        self.lh_pos = [0]*6; self.rh_pos = [0]*6
        rospy.init_node("step_infer", anonymous=True)
        rospy.Subscriber("/l_arm/joint_states", JointState, lambda m: self._cb(m,"l"))
        rospy.Subscriber("/r_arm/joint_states", JointState, lambda m: self._cb(m,"r"))
        if HAS_HAND:
            rospy.Subscriber("/l_arm/rm_driver/Udp_Hand_Status", Hand_Status,
                             lambda m: self._cb_h(m,"l"))
            rospy.Subscriber("/r_arm/rm_driver/Udp_Hand_Status", Hand_Status,
                             lambda m: self._cb_h(m,"r"))

        # ── 臂控: ROS ──
        self.lp = rospy.Publisher("/l_arm/rm_driver/JointPos", JointPos, queue_size=1)
        self.rp = rospy.Publisher("/r_arm/rm_driver/JointPos", JointPos, queue_size=1)

        # ── 手控: TCP (同 deploy_test) ──
        self.ls = self._tcp(LEFT_IP); self.rs = self._tcp(RIGHT_IP)
        # 初始化: 四指张开, 拇指不动
        self.ls.send(b'{"command":"hand_follow_pos","hand_pos":[0,0,0,0,0,0]}\r\n')
        self.rs.send(b'{"command":"hand_follow_pos","hand_pos":[0,0,0,0,0,0]}\r\n')
        print("[手] 四指张开, 拇指不动")

        # 待发动作缓存
        self._pending = None
        rospy.sleep(0.5)

    def _tcp(self, ip):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ip, ARM_PORT))
        s.send(CMD_MODBUS); s.send(CMD_RM_PLUS)
        return s

    def _cb(self, m, s):
        arr = self.l_pos if s=="l" else self.r_pos
        arr[:] = m.position[:6]
    def _cb_h(self, m, s):
        arr = self.lh_pos if s=="l" else self.rh_pos
        arr[:] = m.hand_pos[:6]

    def get_state(self):
        return np.array(self.l_pos + self.r_pos + self.lh_pos + self.rh_pos,
                        dtype=np.float32)

    def predict(self):
        """推理: 返回 (归一化action, 反归一化action)"""
        raw = self.get_state()
        s = torch.tensor(raw, device=self.device).unsqueeze(0)  # (1,24)
        s_norm = (s - self.s_mean) / self.s_std

        with torch.no_grad():
            a_norm = self.policy.predict(s_norm)[0]  # (50,24) → 取第1帧
        a_np = a_norm.cpu().numpy().flatten()
        a_raw = a_np * self.a_std.cpu().numpy().flatten() + self.a_mean.cpu().numpy().flatten()
        return a_np, a_raw

    def send(self, action):
        """action: (24,) 反归一化值, 顺序: 左臂左指 右臂右指"""
        a = action.flatten().tolist() if hasattr(action, 'flatten') else list(action)
        if self.side in ("left", "both"):
            lj = JointPos(); lj.joint = a[0:6]; lj.expand = 0.0; self.lp.publish(lj)
            lh = [max(0,min(65535,int(x))) for x in a[6:12]]
            self.ls.send(f'{{"command":"hand_follow_pos","hand_pos":{lh}}}\r\n'.encode())
        if self.side in ("right", "both"):
            rj = JointPos(); rj.joint = a[12:18]; rj.expand = 0.0; self.rp.publish(rj)
            rh = [max(0,min(65535,int(x))) for x in a[18:24]]
            self.rs.send(f'{{"command":"hand_follow_pos","hand_pos":{rh}}}\r\n'.encode())

    def show_state(self):
        la = [f"{j*57.3:+6.1f}" for j in self.l_pos]
        ra = [f"{j*57.3:+6.1f}" for j in self.r_pos]
        print(f"  左臂: [{', '.join(la)}]°")
        print(f"  右臂: [{', '.join(ra)}]°")
        print(f"  左手: {[int(x) for x in self.lh_pos]}")
        print(f"  右手: {[int(x) for x in self.rh_pos]}")

    def run(self, continuous=False):
        if continuous:
            self._run_continuous()
        else:
            self._run_step()

    def _run_step(self):
        print("\n╔══════════════════════════════════════╗")
        print("║  单步推理调试                        ║")
        print("║  [Enter] 预测 → 打印原始+反归一化值  ║")
        print("║  [Enter] 发送                        ║")
        print("║  [s] 看状态  [q] 退出                ║")
        print("╚══════════════════════════════════════╝\n")

        while not rospy.is_shutdown():
            try:
                cmd = input("> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if cmd == 'q': break
            if cmd == 's':
                self.show_state(); continue

            if self._pending is None:
                a_n, a_r = self.predict()
                self._pending = a_r
                an = a_n.tolist(); ar = a_r.tolist()
                print(f"\n  === 归一化预测 (24D) ===")
                print(f"  左臂: {[f'{x:+7.4f}' for x in an[0:6]]}")
                print(f"  右臂: {[f'{x:+7.4f}' for x in an[12:18]]}")
                print(f"  左手: {[f'{x:+7.4f}' for x in an[6:12]]}")
                print(f"  右手: {[f'{x:+7.4f}' for x in an[18:24]]}")
                print(f"\n  === 反归一化 (真实值) ===")
                print(f"  左臂°: {[f'{x*57.3:+7.1f}' for x in ar[0:6]]}")
                print(f"  左臂rad: {[f'{x:+.4f}' for x in ar[0:6]]}")
                print(f"  右臂°: {[f'{x*57.3:+7.1f}' for x in ar[12:18]]}")
                print(f"  右臂rad: {[f'{x:+.4f}' for x in ar[12:18]]}")
                print(f"  左手raw: {[int(x) for x in ar[6:12]]}")
                print(f"  右手raw: {[int(x) for x in ar[18:24]]}")
                print(f"\n  按 Enter 发送, s 跳过")
            else:
                self.send(self._pending)
                self._pending = None
                print("  → 已发送")
        print("退出")

    def _run_continuous(self):
        period = 1.0 / 25  # 25Hz = 一半速度
        print(f"\n  连续推理 @25Hz (一半速度), 每10帧打印状态, Ctrl+C 退出\n")
        step = 0
        while not rospy.is_shutdown():
            t0 = time.time()
            _, a_r = self.predict()
            self.send(a_r)
            step += 1
            if step % 10 == 0:
                ar = a_r.flatten().tolist()
                info = ""
                if self.side in ("left", "both"):
                    info += f"左臂°: {[f'{x*57.3:+6.1f}' for x in ar[0:6]]}  "
                if self.side in ("right", "both"):
                    info += f"右臂°: {[f'{x*57.3:+6.1f}' for x in ar[12:18]]}  "
                print(f"[{step:5d}] {info}")
            dt = time.time() - t0
            sleep_t = period - dt
            if sleep_t > 0: time.sleep(sleep_t)


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--model", type=str, default="policy_best.pt")
    p.add_argument("--norm", type=str, default="norm_params.pt")
    p.add_argument("--run", action="store_true", help="连续运行@25Hz")
    p.add_argument("--side", type=str, default="both", choices=["left","right","both"],
                        help="只控哪侧: left/right/both")
    args = p.parse_args()

    si = StepInference(model_path=args.model, norm_path=args.norm,
                       side=args.side)
    si.run(continuous=args.run)
