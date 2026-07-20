#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""seg_node.py — 实时语义分割节点(真机唯一缺失的一环)。

契约(smc_live_node 只认这个, 换任何模型都行):
  订阅  <rgb_topic>      sensor_msgs/Image  rgb8/bgr8
  发布  <seg_topic>      sensor_msgs/Image  **mono8**, 每像素 = 类别 id,
        且 header.stamp **原样复制 RGB 的 stamp**(建图节点靠 stamp 与 depth 同步),
        分辨率必须与 depth 一致(不一致时本节点自动最近邻缩放)。

后端可选:
  --backend onnx     ONNX Runtime(默认; CPU/GPU/TensorRT EP 都行)
  --backend torch    PyTorch(torchvision segmentation)
  --backend dummy    不推理, 输出全 0(仅用于打通链路/测吞吐)

标签映射: 模型输出的 train-id 需映射到 labelspace 的类别 id。
  --label-map map.json   形如 {"0": 31, "1": 43, ...}  (模型类别 -> labelspace 类别)
  不给则直接用模型输出的 id。

例:
  python3 seg_node.py --backend onnx --model seg.onnx --label-map map.json \
      --rgb-topic /camera/color/image_raw --seg-topic /seg/labels --size 512
"""
import argparse
import json
import time

import numpy as np


def build_argparser():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rgb-topic", default="/camera/color/image_raw")
    ap.add_argument("--seg-topic", default="/seg/labels")
    ap.add_argument("--backend", choices=["onnx", "torch", "dummy"], default="onnx")
    ap.add_argument("--model", default="", help="onnx/torch 模型路径")
    ap.add_argument("--label-map", default="", help="模型类别 -> labelspace 类别 的 json")
    ap.add_argument("--size", type=int, default=512, help="推理输入短边(缩放)")
    ap.add_argument("--providers", default="CPUExecutionProvider",
                    help="onnxruntime EP, 逗号分隔; 如 CUDAExecutionProvider,CPUExecutionProvider")
    ap.add_argument("--mean", default="0.485,0.456,0.406")
    ap.add_argument("--std", default="0.229,0.224,0.225")
    ap.add_argument("--skip", type=int, default=0, help="每 N 帧推理一次(0=每帧), 复用上次结果")
    return ap


class Backend:
    """把 RGB(H,W,3 uint8) -> 类别图(H,W uint8)。"""

    def __init__(self, a):
        self.a = a
        self.lut = None
        if a.label_map:
            m = json.load(open(a.label_map))
            self.lut = np.zeros(256, np.uint8)
            for k, v in m.items():
                self.lut[int(k)] = int(v)
        self.mean = np.array([float(x) for x in a.mean.split(",")], np.float32)
        self.std = np.array([float(x) for x in a.std.split(",")], np.float32)
        self.sess = None
        if a.backend == "onnx":
            import onnxruntime as ort
            self.sess = ort.InferenceSession(a.model,
                                             providers=a.providers.split(","))
            self.iname = self.sess.get_inputs()[0].name
        elif a.backend == "torch":
            import torch
            self.torch = torch
            self.net = torch.jit.load(a.model) if a.model.endswith(".pt") else torch.load(a.model)
            self.net.eval()

    def _pre(self, rgb):
        import cv2
        h, w = rgb.shape[:2]
        s = self.a.size / min(h, w)
        img = cv2.resize(rgb, (int(round(w * s)), int(round(h * s))), interpolation=cv2.INTER_LINEAR)
        x = img.astype(np.float32) / 255.0
        x = (x - self.mean) / self.std
        return np.transpose(x, (2, 0, 1))[None]      # NCHW

    def infer(self, rgb):
        import cv2
        H, W = rgb.shape[:2]
        if self.a.backend == "dummy":
            return np.zeros((H, W), np.uint8)
        x = self._pre(rgb)
        if self.a.backend == "onnx":
            out = self.sess.run(None, {self.iname: x})[0]
        else:
            with self.torch.no_grad():
                y = self.net(self.torch.from_numpy(x))
                out = (y["out"] if isinstance(y, dict) else y).numpy()
        cls = out[0].argmax(0).astype(np.uint8) if out.ndim == 4 else out[0].astype(np.uint8)
        cls = cv2.resize(cls, (W, H), interpolation=cv2.INTER_NEAREST)   # 回到原分辨率
        return self.lut[cls] if self.lut is not None else cls


def main():
    a = build_argparser().parse_args()
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    be = Backend(a)

    class SegNode(Node):
        def __init__(self):
            super().__init__("seg_node")
            self.pub = self.create_publisher(Image, a.seg_topic, 5)
            self.create_subscription(Image, a.rgb_topic, self.on_rgb, 5)
            self.n = 0
            self.t_sum = 0.0
            self.last = None
            self.get_logger().info("分割节点: %s -> %s  backend=%s model=%s"
                                   % (a.rgb_topic, a.seg_topic, a.backend, a.model or "-"))

        def on_rgb(self, m):
            if m.encoding not in ("rgb8", "bgr8"):
                self.get_logger().error("需要 rgb8/bgr8, 收到 %s" % m.encoding)
                return
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            if m.encoding == "bgr8":
                img = img[:, :, ::-1]
            t0 = time.time()
            if a.skip and self.last is not None and self.n % (a.skip + 1) != 0:
                cls = self.last
            else:
                cls = be.infer(np.ascontiguousarray(img))
                self.last = cls
            self.t_sum += time.time() - t0
            self.n += 1
            out = Image()
            out.header = m.header          # ★ stamp 原样复制, 建图节点靠它与 depth 同步
            out.height, out.width = cls.shape
            out.encoding = "mono8"
            out.step = out.width
            out.is_bigendian = 0
            out.data = cls.tobytes()
            self.pub.publish(out)
            if self.n % 30 == 0:
                self.get_logger().info("frames=%d  %.1f ms/帧  类别数=%d"
                                       % (self.n, 1000 * self.t_sum / self.n, len(np.unique(cls))))

    rclpy.init()
    node = SegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
