#!/usr/bin/env python3
"""
LeRobot 官方 ACT 训练 (含图像)
===============================
用法: conda activate lerobot && python train_lerobot.py
"""

import torch, os, argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=str, default="$HOME/lerobot_dataset")
    parser.add_argument("--ckpt_dir", type=str, default="$HOME/lerobot_trained_act")
    parser.add_argument("--batch_size", type=int, default=2)
    parser.add_argument("--num_epochs", type=int, default=2000)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--kl_weight", type=float, default=0.1)
    parser.add_argument("--chunk_size", type=int, default=50)
    parser.add_argument("--hidden_dim", type=int, default=256)
    parser.add_argument("--dim_feedforward", type=int, default=1024)
    args = parser.parse_args()
    args.dataset = os.path.expanduser(args.dataset)
    args.ckpt_dir = os.path.expanduser(args.ckpt_dir)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"设备: {device}")

    # 1. 加载数据集 (LeRobot 格式, 含图像)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    print(f"加载数据: {args.dataset}")
    dataset = LeRobotDataset(repo_id=args.dataset, delta_timestamps={"observation.state": [0], "action": [0]})
    print(f"  帧数: {len(dataset)}, 图像shape: {dataset[0]['observation.images.cam_head'].shape}")

    # 2. ACT 策略配置
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy

    config = ACTConfig(
        n_obs_steps=1,
        chunk_size=args.chunk_size,
        input_shapes=dataset.input_shapes,
        output_shapes=dataset.output_shapes,
        hidden_dim=args.hidden_dim,
        dim_feedforward=args.dim_feedforward,
        kl_weight=args.kl_weight,
    )
    policy = ACTPolicy(config).to(device)
    policy.train()
    print(f"  参数: {sum(p.numel() for p in policy.parameters()):,}")

    # 3. 训练循环
    from torch.utils.data import DataLoader
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True,
                        num_workers=0, pin_memory=True)
    optimizer = torch.optim.Adam(policy.parameters(), lr=args.lr)
    os.makedirs(args.ckpt_dir, exist_ok=True)
    best_loss = float('inf')

    print(f"开始训练 {args.num_epochs} 轮...")
    for epoch in range(args.num_epochs):
        policy.train()
        total = 0.0; n = 0
        for batch in loader:
            batch = {k: v.to(device) if isinstance(v, torch.Tensor) else v
                     for k, v in batch.items()}
            loss, _ = policy.forward(batch)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total += loss.item(); n += 1

        avg = total / n
        print(f"Epoch {epoch:4d}  loss={avg:.4f}", flush=True)

        if avg < best_loss:
            best_loss = avg
            policy.save_pretrained(args.ckpt_dir)
            print(f"  → 保存最佳 (loss={avg:.4f})")

    print(f"\n训练完成, 最佳 loss={best_loss:.4f}")


if __name__ == "__main__":
    main()
