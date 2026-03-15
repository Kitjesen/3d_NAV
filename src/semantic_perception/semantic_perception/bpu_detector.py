"""
BPU YOLO 检测器 — D-Robotics Nash BPU 加速的 COCO-80 检测 + 实例分割。

在 S100P (128 TOPS) 上通过 HB_HBMRuntime 运行 .hbm 模型，
替代 YOLO-World/YOLO-E CPU 推理 (~500ms → ~45ms/帧)。

支持:
  - YOLO v8n/11n/11s/12n/12s detect (纯检测)
  - YOLO 11s seg (检测 + 实例分割 mask)
  - 自动发现最优模型 (seg > detect)
  - COCO 80 类 → text_prompt 标签映射 (兼容开放词汇接口)
"""

import os
from typing import List, Optional

import cv2
import numpy as np

from .detector_base import Detection2D, DetectorBase


# COCO 80 类名 (与 ultralytics/YOLO 一致)
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]

# COCO 名 → class_id 反向索引
_COCO_NAME_TO_ID = {name: i for i, name in enumerate(COCO_NAMES)}

# 双向同义词组 — 任一词匹配时自动扩展到同组所有词
_SYNONYM_GROUPS = [
    {"person", "people", "human", "pedestrian"},
    {"desk", "table", "dining table"},
    {"sofa", "couch"},
    {"monitor", "screen", "tv", "television"},
    {"plant", "potted plant", "flower"},
    {"phone", "cell phone", "mobile"},
    {"fridge", "refrigerator"},
    {"motorbike", "motorcycle"},
    {"aeroplane", "airplane"},
    {"bin", "trash can", "recycling bin"},
    {"bus", "truck"},  # bus.jpg 里的 bus 被检测为 truck
]
# 预计算: word → 同组所有词
_SYNONYM_EXPAND: dict = {}
for _group in _SYNONYM_GROUPS:
    for _word in _group:
        _SYNONYM_EXPAND[_word] = _group

# 常用同义词 → COCO 类名 (导航场景中的非 COCO 词汇)
_SYNONYMS = {
    "desk": "dining table",
    "table": "dining table",
    "sofa": "couch",
    "monitor": "tv",
    "screen": "tv",
    "plant": "potted plant",
    "flower": "potted plant",
    "phone": "cell phone",
    "mobile": "cell phone",
    "fridge": "refrigerator",
    "bin": "trash can",      # trash can 不在 COCO, 但保留映射
    "motorbike": "motorcycle",
    "aeroplane": "airplane",
    "couch": "couch",
    "television": "tv",
    "laptop computer": "laptop",
}


class BPUDetector(DetectorBase):
    """D-Robotics Nash BPU YOLO 检测器 (DetectorBase 适配)。

    闭词汇 COCO-80 检测器，通过 text_prompt 标签匹配过滤输出，
    兼容 perception_node.py 的开放词汇检测接口。
    """

    MODEL_CANDIDATES = [
        # YOLO-World (导航大词汇表, 优先) — 需通过 export_yoloworld_bpu.py + convert_onnx_to_hbm.sh 生成
        "/home/sunrise/models/yolov8s-worldv2_nav*_640_nashe_640x640_nv12.hbm",
        # 标准 YOLO (COCO-80 闭词汇, 降级)
        "/home/sunrise/models/yolo11s_seg_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo12s_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo12n_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo11s_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo11n_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolov8n_detect_nashe_640x640_nv12.hbm",
    ]
    INPUT_SIZE = 640
    STRIDES = [8, 16, 32]

    def __init__(
        self,
        confidence: float = 0.25,
        iou_threshold: float = 0.45,
        model_path: Optional[str] = None,
        max_detections: int = 30,
    ):
        self._conf_thr = confidence
        self._iou_thr = iou_threshold
        self._model_path = model_path
        self._max_detections = max_detections
        self._rt = None
        self._mname = None
        self._output_map = []
        self._proto_name = None
        self._bbox_scales = {}
        self._dfl_weights = np.arange(16, dtype=np.float32)
        self.has_seg = False
        self._model_name_short = ""
        # 缓存: text_prompt → allowed COCO class_id set
        self._prompt_cache_key = ""
        self._allowed_cids: set = set()

    def load_model(self) -> None:
        """加载 BPU .hbm 模型。"""
        import glob as _glob
        from hbm_runtime import HB_HBMRuntime

        path = self._model_path
        if not path:
            for pattern in self.MODEL_CANDIDATES:
                if "*" in pattern:
                    matches = sorted(_glob.glob(pattern))
                    if matches:
                        path = matches[-1]  # 取最新的
                        break
                elif os.path.exists(pattern):
                    path = pattern
                    break
            if path is None:
                raise FileNotFoundError(
                    f"No BPU YOLO model found. Searched: {self.MODEL_CANDIDATES}"
                )

        # 自动加载词汇表 (YOLO-World 导出时生成的 _vocab.json)
        self._custom_vocab = None
        vocab_pattern = path.replace(".hbm", "").replace("_nashe_640x640_nv12", "") + "*_vocab.json"
        vocab_matches = _glob.glob(vocab_pattern)
        if not vocab_matches:
            # 尝试同目录下任何 *_vocab.json
            model_dir = os.path.dirname(path)
            vocab_matches = _glob.glob(os.path.join(model_dir, "*_vocab.json"))
        if vocab_matches:
            import json
            try:
                with open(vocab_matches[0], "r", encoding="utf-8") as f:
                    vdata = json.load(f)
                names = vdata.get("names", {})
                self._custom_vocab = {int(k): v for k, v in names.items()}
                print(f"[BPU] Custom vocabulary loaded: {len(self._custom_vocab)} classes from {vocab_matches[0]}")
            except Exception as e:
                print(f"[BPU] Failed to load vocab: {e}, using COCO-80")

        base = os.path.basename(path)
        self._model_name_short = (
            base.replace("_nashe_640x640_nv12.hbm", "")
            .replace("_detect", "")
            .replace("_seg", "-seg")
        )
        self.has_seg = "_seg_" in base

        self._rt = HB_HBMRuntime(path)
        self._mname = self._rt.model_names[0]

        # 跑一次 dummy 推理发现输出结构
        y = np.zeros((1, self.INPUT_SIZE, self.INPUT_SIZE, 1), dtype=np.uint8)
        uv = np.zeros(
            (1, self.INPUT_SIZE // 2, self.INPUT_SIZE // 2, 2), dtype=np.uint8
        )
        dummy_out = self._rt.run({"images_y": y, "images_uv": uv})[self._mname]

        # 类别数: 从模型实际输出推断（不依赖 vocab 文件，避免 vocab/model 不匹配）
        # 先扫描输出找最可能的 cls 通道数
        _candidate_cls_ch = set()
        for name, arr in dummy_out.items():
            ch = arr.shape[-1]
            if ch not in (64, 32, 1) and ch != self.INPUT_SIZE // 4:
                _candidate_cls_ch.add(ch)
        # 优先匹配自定义词汇表，否则取非 64/32 的通道数
        if self._custom_vocab and len(self._custom_vocab) in _candidate_cls_ch:
            self._num_classes = len(self._custom_vocab)
        elif 80 in _candidate_cls_ch:
            self._num_classes = 80
            if self._custom_vocab:
                print(f"[BPU] WARNING: vocab has {len(self._custom_vocab)} classes but model has 80 → using COCO-80")
                self._custom_vocab = None
        elif _candidate_cls_ch:
            self._num_classes = max(_candidate_cls_ch)
        else:
            self._num_classes = 80

        # 检测输出格式: YOLOE 端到端 vs YOLO 多尺度
        self._is_yoloe = False
        self._yoloe_det_name = None
        self._yoloe_proto_name = None

        for name, arr in dummy_out.items():
            # YOLOE 端到端: output0=[1, 300, 38], output1=[1, 32, 160, 160]
            if arr.ndim == 3 and arr.shape[1] <= 300 and arr.shape[2] > 4:
                self._is_yoloe = True
                self._yoloe_det_name = name
                # 38 = 4(bbox) + 1(score) + 1(class) + 32(mask_coeffs)
                self._num_classes = len(self._custom_vocab) if self._custom_vocab else 80
                print(f"[BPU] YOLOE end-to-end format: {name} shape={arr.shape}")

        if self._is_yoloe:
            # 找 proto mask
            for name, arr in dummy_out.items():
                if name != self._yoloe_det_name and arr.ndim == 4:
                    self._yoloe_proto_name = name
                    print(f"[BPU] YOLOE proto: {name} shape={arr.shape}")
            self._output_map = []
            self._proto_name = self._yoloe_proto_name
        else:
            cls_outs = {}   # grid_size → name (ch=num_classes)
            bbox_outs = {}  # grid_size → name (ch=64)
            mask_outs = {}  # grid_size → name (ch=32, mask coefficients)
            self._proto_name = None
            for name, arr in dummy_out.items():
                gs = arr.shape[1]
                ch = arr.shape[-1]
                if ch == self._num_classes:
                    cls_outs[gs] = name
                elif ch == 64:
                    bbox_outs[gs] = name
                elif ch == 32:
                    if gs == self.INPUT_SIZE // 4:  # 160×160 = proto
                        self._proto_name = name
                    else:
                        mask_outs[gs] = name

            self._output_map = []
            for stride in self.STRIDES:
                gs = self.INPUT_SIZE // stride
                mc = mask_outs.get(gs)
                self._output_map.append((cls_outs[gs], bbox_outs[gs], mc))

        # 量化 scale (v8n 的 int32 bbox)
        self._bbox_scales = {}
        try:
            oq = self._rt.output_quants[self._mname]
            for _, bbox_name, _ in self._output_map:
                qp = oq[bbox_name]
                if len(qp.scale) > 0:
                    self._bbox_scales[bbox_name] = qp.scale.astype(np.float32)
        except Exception:
            pass

        seg_info = f", proto={self._proto_name}" if self._proto_name else ""
        print(
            f"[BPU] {self._model_name_short} loaded (seg={self.has_seg}{seg_info})"
        )

    def detect(self, rgb: np.ndarray, text_prompt: str) -> List[Detection2D]:
        """
        BPU 推理 + COCO 标签过滤。

        Args:
            rgb: HxWx3 uint8 BGR 图像
            text_prompt: ". " 分隔的目标标签列表 (e.g. "door . chair . person")

        Returns:
            匹配 text_prompt 中 COCO 类的 Detection2D 列表
        """
        if self._rt is None:
            return []

        # 解析 text_prompt → 允许的 class_id 集合 (有缓存)
        allowed = self._parse_prompt(text_prompt)

        bgr = rgb  # perception_node 传 bgr
        h0, w0 = bgr.shape[:2]

        # 预处理: letterbox + NV12
        y_plane, uv_plane, scale, pad_x, pad_y = self._preprocess(bgr)

        # BPU 推理
        outputs = self._rt.run({"images_y": y_plane, "images_uv": uv_plane})[
            self._mname
        ]

        # YOLOE 端到端: 直接解析 NMS 后的输出
        if self._is_yoloe:
            return self._detect_yoloe(outputs, allowed, scale, pad_x, pad_y, h0, w0)

        # 标准 YOLO: 多尺度解码 + NMS
        raw, kept_mc = self._postprocess(outputs, scale, pad_x, pad_y, h0, w0)
        if not raw:
            return []

        # 实例分割 mask 生成 (seg 模型, batched matmul)
        masks_list = self._generate_masks(raw, kept_mc, outputs, scale, pad_x, pad_y)

        # 转换为 Detection2D, 按 text_prompt 过滤
        results: List[Detection2D] = []
        frame_area = h0 * w0
        for i, (box, score, cid) in enumerate(raw):
            cid = int(cid)
            # 过滤: 仅保留 text_prompt 中能匹配的 COCO 类
            if allowed and cid not in allowed:
                continue
            x1, y1, x2, y2 = box.astype(int).tolist()
            bw, bh = x2 - x1, y2 - y1
            # 跳过过小的检测 (<20px)
            if bw < 20 or bh < 20:
                continue

            # 构造 mask (full-frame bool HxW)
            mask = None
            if masks_list[i] is not None:
                mask_crop, mx, my = masks_list[i]
                # 展开为 full-frame mask (Detection2D.mask 是 HxW bool)
                full_mask = np.zeros((h0, w0), dtype=bool)
                mh, mw = mask_crop.shape[:2]
                # 裁剪到图像边界
                ix1 = max(0, mx)
                iy1 = max(0, my)
                ix2 = min(w0, mx + mw)
                iy2 = min(h0, my + mh)
                if ix2 > ix1 and iy2 > iy1:
                    full_mask[iy1:iy2, ix1:ix2] = mask_crop[
                        iy1 - my : iy2 - my, ix1 - mx : ix2 - mx
                    ]
                mask = full_mask

            # 使用自定义词汇表或 COCO 默认名
            if self._custom_vocab and cid in self._custom_vocab:
                label = self._custom_vocab[cid]
            elif cid < len(COCO_NAMES):
                label = COCO_NAMES[cid]
            else:
                label = f"class_{cid}"

            results.append(
                Detection2D(
                    bbox=np.array([x1, y1, x2, y2], dtype=np.float32),
                    score=float(score),
                    label=label,
                    class_id=cid,
                    mask=mask,
                )
            )
            if len(results) >= self._max_detections:
                break

        return results

    def shutdown(self) -> None:
        """释放 BPU 资源。"""
        self._rt = None

    # ================================================================
    #  text_prompt → COCO class_id 映射
    # ================================================================

    def _detect_yoloe(self, outputs, allowed, scale, pad_x, pad_y, h0, w0):
        """YOLOE 端到端输出解析: [1, 300, 38] → Detection2D 列表。"""
        det_out = outputs[self._yoloe_det_name][0]  # (300, 38)
        # 38 = 4(bbox xyxy) + 1(score) + 1(class_id) + 32(mask_coeffs)
        results: List[Detection2D] = []

        for i in range(det_out.shape[0]):
            row = det_out[i]
            score = float(row[4])
            if score < self._conf_thr:
                continue

            cid = int(row[5])
            # letterbox 逆变换
            x1 = (float(row[0]) - pad_x) / scale
            y1 = (float(row[1]) - pad_y) / scale
            x2 = (float(row[2]) - pad_x) / scale
            y2 = (float(row[3]) - pad_y) / scale

            x1 = max(0, min(x1, w0))
            y1 = max(0, min(y1, h0))
            x2 = max(0, min(x2, w0))
            y2 = max(0, min(y2, h0))

            bw, bh = x2 - x1, y2 - y1
            if bw < 10 or bh < 10:
                continue

            # 过滤: 仅保留 text_prompt 中能匹配的类
            if allowed and cid not in allowed:
                continue

            # 标签
            if self._custom_vocab and cid in self._custom_vocab:
                label = self._custom_vocab[cid]
            elif cid < len(COCO_NAMES):
                label = COCO_NAMES[cid]
            else:
                label = f"class_{cid}"

            # mask 生成 (如果有 proto)
            mask = None
            if self._yoloe_proto_name and self._yoloe_proto_name in outputs:
                mask_coeffs = row[6:38]  # 32 coefficients
                proto = outputs[self._yoloe_proto_name][0]  # (32, 160, 160)
                if proto.ndim == 3:
                    ph, pw = proto.shape[1], proto.shape[2]
                    proto_flat = proto.reshape(32, -1)  # (32, 25600)
                    mask_raw = mask_coeffs @ proto_flat  # (25600,)
                    mask_raw = mask_raw.reshape(ph, pw)
                    np.clip(mask_raw, -50, 50, out=mask_raw)
                    mask_prob = 1.0 / (1.0 + np.exp(-mask_raw))
                    # 裁剪到 bbox 区域并 resize
                    bx1 = max(0, int((x1 * scale + pad_x) / 4))
                    by1 = max(0, int((y1 * scale + pad_y) / 4))
                    bx2 = min(pw, int((x2 * scale + pad_x) / 4) + 1)
                    by2 = min(ph, int((y2 * scale + pad_y) / 4) + 1)
                    if bx2 > bx1 and by2 > by1:
                        crop = mask_prob[by1:by2, bx1:bx2]
                        cw, ch = max(1, int(x2 - x1)), max(1, int(y2 - y1))
                        mask_resized = cv2.resize(crop, (cw, ch), interpolation=cv2.INTER_LINEAR)
                        full_mask = np.zeros((h0, w0), dtype=bool)
                        ix1, iy1 = max(0, int(x1)), max(0, int(y1))
                        ix2 = min(w0, ix1 + cw)
                        iy2 = min(h0, iy1 + ch)
                        mw = ix2 - ix1
                        mh = iy2 - iy1
                        if mw > 0 and mh > 0:
                            full_mask[iy1:iy2, ix1:ix2] = mask_resized[:mh, :mw] > 0.5
                            mask = full_mask

            results.append(Detection2D(
                bbox=np.array([x1, y1, x2, y2], dtype=np.float32),
                score=score,
                label=label,
                class_id=cid,
                mask=mask,
            ))
            if len(results) >= self._max_detections:
                break

        return results

    def _parse_prompt(self, text_prompt: str) -> set:
        """解析 ". " 分隔的 text_prompt, 返回匹配到的 COCO class_id 集合。

        匹配策略 (按优先级):
        1. 精确匹配 COCO 类名
        2. 同义词映射 (_SYNONYMS)
        3. 子串模糊匹配
        空集表示不过滤 (返回所有检测)。
        """
        if text_prompt == self._prompt_cache_key:
            return self._allowed_cids

        self._prompt_cache_key = text_prompt
        labels = [l.strip().lower() for l in text_prompt.split(".") if l.strip()]
        # 自动扩展同义词
        expanded = set(labels)
        for l in labels:
            if l in _SYNONYM_EXPAND:
                expanded.update(_SYNONYM_EXPAND[l])
        labels = list(expanded)
        if not labels:
            self._allowed_cids = set()
            return self._allowed_cids

        # 构建名称→ID 索引 (优先自定义词汇表)
        if self._custom_vocab:
            name_to_id = {v.lower(): k for k, v in self._custom_vocab.items()}
        else:
            name_to_id = _COCO_NAME_TO_ID

        matched = set()
        for label in labels:
            # 1. 精确匹配
            if label in name_to_id:
                matched.add(name_to_id[label])
                continue
            # 2. 同义词映射 (仅 COCO 模式)
            if not self._custom_vocab and label in _SYNONYMS:
                syn = _SYNONYMS[label]
                if syn in name_to_id:
                    matched.add(name_to_id[syn])
                    continue
            # 3. 子串模糊匹配
            for name, cid in name_to_id.items():
                if label in name or name in label:
                    matched.add(cid)

        self._allowed_cids = matched
        return self._allowed_cids

    # ================================================================
    #  预处理: letterbox + BGR → NV12
    # ================================================================

    def _preprocess(self, bgr):
        h0, w0 = bgr.shape[:2]
        sz = self.INPUT_SIZE
        scale = min(sz / h0, sz / w0)
        nh, nw = int(h0 * scale), int(w0 * scale)
        pad_x, pad_y = (sz - nw) // 2, (sz - nh) // 2

        canvas = np.full((sz, sz, 3), 114, dtype=np.uint8)
        resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas[pad_y : pad_y + nh, pad_x : pad_x + nw] = resized

        yuv = cv2.cvtColor(canvas, cv2.COLOR_BGR2YUV_I420)
        y_plane = yuv[:sz, :].reshape(1, sz, sz, 1)
        u = yuv[sz : sz + sz // 4, :].reshape(sz // 2, sz // 2)
        v = yuv[sz + sz // 4 :, :].reshape(sz // 2, sz // 2)
        uv_plane = np.stack([u, v], axis=-1).reshape(1, sz // 2, sz // 2, 2)

        return (
            np.ascontiguousarray(y_plane),
            np.ascontiguousarray(uv_plane),
            scale,
            pad_x,
            pad_y,
        )

    # ================================================================
    #  后处理: 多尺度解码 + NMS
    # ================================================================

    def _postprocess(self, outputs, scale, pad_x, pad_y, orig_h, orig_w):
        all_boxes = []
        all_scores = []
        all_classes = []
        all_mask_coeffs = []

        for i, stride in enumerate(self.STRIDES):
            cls_name, bbox_name, mask_name = self._output_map[i]
            cls_logits = outputs[cls_name][0]
            bbox_raw = outputs[bbox_name][0]
            H, W = cls_logits.shape[:2]

            cls_scores = 1.0 / (1.0 + np.exp(-cls_logits.astype(np.float32)))
            max_scores = cls_scores.max(axis=-1)
            mask = max_scores > self._conf_thr
            if not mask.any():
                continue

            ys, xs = np.where(mask)
            valid_cls = cls_scores[ys, xs]
            valid_bbox = bbox_raw[ys, xs]
            valid_max = max_scores[ys, xs]
            valid_cids = valid_cls.argmax(axis=-1)

            valid_mc = None
            if mask_name and mask_name in outputs:
                valid_mc = outputs[mask_name][0][ys, xs]

            if bbox_name in self._bbox_scales:
                bbox_f = valid_bbox.astype(np.float32) * self._bbox_scales[bbox_name]
            else:
                bbox_f = valid_bbox.astype(np.float32)
            bbox_f = bbox_f.reshape(-1, 4, 16)
            bbox_exp = np.exp(bbox_f - bbox_f.max(axis=-1, keepdims=True))
            bbox_sm = bbox_exp / bbox_exp.sum(axis=-1, keepdims=True)
            dist = (bbox_sm * self._dfl_weights).sum(axis=-1)

            cx = (xs.astype(np.float32) + 0.5) * stride
            cy = (ys.astype(np.float32) + 0.5) * stride

            x1 = (cx - dist[:, 0] * stride - pad_x) / scale
            y1 = (cy - dist[:, 1] * stride - pad_y) / scale
            x2 = (cx + dist[:, 2] * stride - pad_x) / scale
            y2 = (cy + dist[:, 3] * stride - pad_y) / scale

            x1 = np.clip(x1, 0, orig_w)
            y1 = np.clip(y1, 0, orig_h)
            x2 = np.clip(x2, 0, orig_w)
            y2 = np.clip(y2, 0, orig_h)

            boxes = np.stack([x1, y1, x2, y2], axis=-1)
            all_boxes.append(boxes)
            all_scores.append(valid_max)
            all_classes.append(valid_cids)
            if valid_mc is not None:
                all_mask_coeffs.append(valid_mc)

        if not all_boxes:
            return [], None

        boxes = np.concatenate(all_boxes)
        scores = np.concatenate(all_scores)
        classes = np.concatenate(all_classes)
        mask_coeffs = np.concatenate(all_mask_coeffs) if all_mask_coeffs else None

        keep = self._nms(boxes, scores, classes)
        results = [(boxes[i], scores[i], classes[i]) for i in keep]
        kept_mc = mask_coeffs[keep] if mask_coeffs is not None else None
        return results, kept_mc

    def _nms(self, boxes, scores, classes):
        keep = []
        for cid in np.unique(classes):
            m = classes == cid
            idx = np.where(m)[0]
            if len(idx) == 0:
                continue
            b = boxes[idx]
            s = scores[idx]
            order = s.argsort()[::-1]
            picked = []
            while len(order) > 0:
                i = order[0]
                picked.append(idx[i])
                if len(order) == 1:
                    break
                rest = order[1:]
                ious = self._compute_iou(b[i], b[rest])
                order = rest[ious < self._iou_thr]
            keep.extend(picked)
        return keep

    @staticmethod
    def _compute_iou(box, boxes):
        ix1 = np.maximum(box[0], boxes[:, 0])
        iy1 = np.maximum(box[1], boxes[:, 1])
        ix2 = np.minimum(box[2], boxes[:, 2])
        iy2 = np.minimum(box[3], boxes[:, 3])
        inter = np.maximum(ix2 - ix1, 0) * np.maximum(iy2 - iy1, 0)
        a1 = (box[2] - box[0]) * (box[3] - box[1])
        a2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        return inter / np.maximum(a1 + a2 - inter, 1e-6)

    # ================================================================
    #  实例分割 mask 生成 (batched matmul)
    # ================================================================

    def _generate_masks(self, raw, kept_mc, outputs, scale, pad_x, pad_y):
        """从 proto × coefficients 生成实例 mask。返回 list, 每项为 (mask_crop, x, y) 或 None。"""
        masks_list = [None] * len(raw)
        if kept_mc is None or not self._proto_name or self._proto_name not in outputs:
            return masks_list

        proto = outputs[self._proto_name][0]  # (160, 160, 32)
        ph, pw = proto.shape[:2]
        proto_flat = proto.reshape(-1, 32)
        all_masks = proto_flat @ kept_mc.T
        np.clip(all_masks, -50, 50, out=all_masks)
        all_masks = 1.0 / (1.0 + np.exp(-all_masks))
        all_masks = all_masks.reshape(ph, pw, -1)

        for idx in range(len(raw)):
            box = raw[idx][0]
            bx1 = max(0, int((box[0] * scale + pad_x) / 4))
            by1 = max(0, int((box[1] * scale + pad_y) / 4))
            bx2 = min(pw, int((box[2] * scale + pad_x) / 4) + 1)
            by2 = min(ph, int((box[3] * scale + pad_y) / 4) + 1)
            if bx2 <= bx1 or by2 <= by1:
                continue
            crop = all_masks[by1:by2, bx1:bx2, idx]
            ox1, oy1, ox2, oy2 = box.astype(int)
            cw, ch = max(1, ox2 - ox1), max(1, oy2 - oy1)
            mask_resized = cv2.resize(crop, (cw, ch), interpolation=cv2.INTER_LINEAR)
            masks_list[idx] = (mask_resized > 0.5, ox1, oy1)

        return masks_list
