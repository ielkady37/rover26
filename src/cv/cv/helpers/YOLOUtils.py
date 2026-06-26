#!/usr/bin/env python3
"""YOLOPv2 inference utilities: model helpers, NMS, segmentation, and metrics."""

import logging
import os
import platform
import random
import subprocess
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import torch
import torchvision

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _git_describe(path=Path(__file__).parent):
    s = f'git -C {path} describe --tags --long --always'
    try:
        return subprocess.check_output(s, shell=True, stderr=subprocess.STDOUT).decode()[:-1]
    except subprocess.CalledProcessError:
        return ''


def _date_modified(path=__file__):
    t = datetime.fromtimestamp(Path(path).stat().st_mtime)
    return f'{t.year}-{t.month}-{t.day}'


def _make_grid(nx=20, ny=20):
    yv, xv = torch.meshgrid([torch.arange(ny), torch.arange(nx)])
    return torch.stack((xv, yv), 2).view((1, 1, ny, nx, 2)).float()


# ---------------------------------------------------------------------------
# Metric helpers
# ---------------------------------------------------------------------------

class AverageMeter:
    """Computes and stores the average and current value."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.val = 0
        self.avg = 0
        self.sum = 0
        self.count = 0

    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
        self.avg = self.sum / self.count if self.count != 0 else 0


class SegmentationMetric:
    """Pixel-level segmentation metric using a confusion matrix.

    confusionMatrix layout::

        [[TN, FP],
         [FN, TP]]
    """

    def __init__(self, numClass):
        self.numClass = numClass
        self.confusionMatrix = np.zeros((self.numClass,) * 2)

    def pixelAccuracy(self):
        """Return overall pixel accuracy (TP+TN) / total."""
        return np.diag(self.confusionMatrix).sum() / self.confusionMatrix.sum()

    def lineAccuracy(self):
        """Return per-class accuracy for the lane-line class (index 1)."""
        acc = np.diag(self.confusionMatrix) / (self.confusionMatrix.sum(axis=1) + 1e-12)
        return acc[1]

    def classPixelAccuracy(self):
        """Return per-class precision."""
        return np.diag(self.confusionMatrix) / (self.confusionMatrix.sum(axis=0) + 1e-12)

    def meanPixelAccuracy(self):
        return np.nanmean(self.classPixelAccuracy())

    def meanIntersectionOverUnion(self):
        intersection = np.diag(self.confusionMatrix)
        union = (
            np.sum(self.confusionMatrix, axis=1)
            + np.sum(self.confusionMatrix, axis=0)
            - np.diag(self.confusionMatrix)
        )
        iou = intersection / union
        iou[np.isnan(iou)] = 0
        return np.nanmean(iou)

    def IntersectionOverUnion(self):
        intersection = np.diag(self.confusionMatrix)
        union = (
            np.sum(self.confusionMatrix, axis=1)
            + np.sum(self.confusionMatrix, axis=0)
            - np.diag(self.confusionMatrix)
        )
        iou = intersection / union
        iou[np.isnan(iou)] = 0
        return iou[1]

    def Frequency_Weighted_Intersection_over_Union(self):
        freq = np.sum(self.confusionMatrix, axis=1) / np.sum(self.confusionMatrix)
        iu = np.diag(self.confusionMatrix) / (
            np.sum(self.confusionMatrix, axis=1)
            + np.sum(self.confusionMatrix, axis=0)
            - np.diag(self.confusionMatrix)
        )
        return (freq[freq > 0] * iu[freq > 0]).sum()

    def genConfusionMatrix(self, imgPredict, imgLabel):
        mask = (imgLabel >= 0) & (imgLabel < self.numClass)
        label = self.numClass * imgLabel[mask] + imgPredict[mask]
        count = np.bincount(label, minlength=self.numClass ** 2)
        return count.reshape(self.numClass, self.numClass)

    def addBatch(self, imgPredict, imgLabel):
        assert imgPredict.shape == imgLabel.shape
        self.confusionMatrix += self.genConfusionMatrix(imgPredict, imgLabel)

    def reset(self):
        self.confusionMatrix = np.zeros((self.numClass, self.numClass))


# ---------------------------------------------------------------------------
# Device / model helpers
# ---------------------------------------------------------------------------

class YOLOUtils:
    """Static helpers for YOLOPv2 inference, NMS, segmentation, and metrics."""

    @staticmethod
    def select_device(device='', batch_size=None):
        """Return a ``torch.device`` for *device* string (e.g. ``'0'`` or ``'cpu'``)."""
        s = f'YOLOPv2 {_git_describe() or _date_modified()} torch {torch.__version__} '
        cpu = device.lower() == 'cpu'
        if cpu:
            os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
        elif device:
            os.environ['CUDA_VISIBLE_DEVICES'] = device
            assert torch.cuda.is_available(), \
                f'CUDA unavailable, invalid device {device} requested'

        cuda = not cpu and torch.cuda.is_available()
        if cuda:
            n = torch.cuda.device_count()
            if n > 1 and batch_size:
                assert batch_size % n == 0, \
                    f'batch-size {batch_size} not multiple of GPU count {n}'
            space = ' ' * len(s)
            for i, d in enumerate(device.split(',') if device else range(n)):
                p = torch.cuda.get_device_properties(i)
                s += f"{'' if i == 0 else space}CUDA:{d} ({p.name}, {p.total_memory / 1024 ** 2}MB)\n"
        else:
            s += 'CPU\n'

        logger.info(s.encode().decode('ascii', 'ignore') if platform.system() == 'Windows' else s)
        return torch.device('cuda:0' if cuda else 'cpu')

    @staticmethod
    def time_synchronized():
        """Return a pytorch-accurate timestamp."""
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        return time.time()

    @staticmethod
    def split_for_trace_model(pred=None, anchor_grid=None):
        """Decode raw trace-model output into standard prediction format."""
        z = []
        st = [8, 16, 32]
        for i in range(3):
            bs, _, ny, nx = pred[i].shape
            pred[i] = pred[i].view(bs, 3, 85, ny, nx).permute(0, 1, 3, 4, 2).contiguous()
            y = pred[i].sigmoid()
            gr = _make_grid(nx, ny).to(pred[i].device)
            y[..., 0:2] = (y[..., 0:2] * 2.0 - 0.5 + gr) * st[i]
            y[..., 2:4] = (y[..., 2:4] * 2) ** 2 * anchor_grid[i]
            z.append(y.view(bs, -1, 85))
        return torch.cat(z, 1)

    # ------------------------------------------------------------------
    # Bounding-box helpers

    @staticmethod
    def xywh2xyxy(x):
        """Convert boxes from [x, y, w, h] to [x1, y1, x2, y2]."""
        y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
        y[:, 0] = x[:, 0] - x[:, 2] / 2
        y[:, 1] = x[:, 1] - x[:, 3] / 2
        y[:, 2] = x[:, 0] + x[:, 2] / 2
        y[:, 3] = x[:, 1] + x[:, 3] / 2
        return y

    @staticmethod
    def xyxy2xywh(x):
        """Convert boxes from [x1, y1, x2, y2] to [x, y, w, h]."""
        y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
        y[:, 0] = (x[:, 0] + x[:, 2]) / 2
        y[:, 1] = (x[:, 1] + x[:, 3]) / 2
        y[:, 2] = x[:, 2] - x[:, 0]
        y[:, 3] = x[:, 3] - x[:, 1]
        return y

    @staticmethod
    def box_iou(box1, box2):
        """Return NxM IoU matrix for two sets of xyxy boxes."""
        def box_area(box):
            return (box[2] - box[0]) * (box[3] - box[1])

        area1 = box_area(box1.T)
        area2 = box_area(box2.T)
        inter = (
            torch.min(box1[:, None, 2:], box2[:, 2:])
            - torch.max(box1[:, None, :2], box2[:, :2])
        ).clamp(0).prod(2)
        return inter / (area1[:, None] + area2 - inter)

    @staticmethod
    def non_max_suppression(
        prediction,
        conf_thres=0.25,
        iou_thres=0.45,
        classes=None,
        agnostic=False,
        multi_label=False,
        labels=(),
    ):
        """Run NMS; returns list of (n,6) tensors [xyxy, conf, cls] per image."""
        nc = prediction.shape[2] - 5
        xc = prediction[..., 4] > conf_thres

        min_wh, max_wh = 2, 4096
        max_det = 300
        max_nms = 30000
        time_limit = 10.0
        redundant = True
        multi_label &= nc > 1
        merge = False

        t = time.time()
        output = [torch.zeros((0, 6), device=prediction.device)] * prediction.shape[0]
        for xi, x in enumerate(prediction):
            x = x[xc[xi]]
            if labels and len(labels[xi]):
                lbl = labels[xi]
                v = torch.zeros((len(lbl), nc + 5), device=x.device)
                v[:, :4] = lbl[:, 1:5]
                v[:, 4] = 1.0
                v[range(len(lbl)), lbl[:, 0].long() + 5] = 1.0
                x = torch.cat((x, v), 0)

            if not x.shape[0]:
                continue

            x[:, 5:] *= x[:, 4:5]
            box = YOLOUtils.xywh2xyxy(x[:, :4])

            if multi_label:
                i, j = (x[:, 5:] > conf_thres).nonzero(as_tuple=False).T
                x = torch.cat((box[i], x[i, j + 5, None], j[:, None].float()), 1)
            else:
                conf, j = x[:, 5:].max(1, keepdim=True)
                x = torch.cat((box, conf, j.float()), 1)[conf.view(-1) > conf_thres]

            if classes is not None:
                x = x[(x[:, 5:6] == torch.tensor(classes, device=x.device)).any(1)]

            n = x.shape[0]
            if not n:
                continue
            elif n > max_nms:
                x = x[x[:, 4].argsort(descending=True)[:max_nms]]

            c = x[:, 5:6] * (0 if agnostic else max_wh)
            boxes, scores = x[:, :4] + c, x[:, 4]
            i = torchvision.ops.nms(boxes, scores, iou_thres)
            if i.shape[0] > max_det:
                i = i[:max_det]
            if merge and (1 < n < 3e3):
                iou = YOLOUtils.box_iou(boxes[i], boxes) > iou_thres
                weights = iou * scores[None]
                x[i, :4] = torch.mm(weights, x[:, :4]).float() / weights.sum(1, keepdim=True)
                if redundant:
                    i = i[iou.sum(1) > 1]

            output[xi] = x[i]
            if (time.time() - t) > time_limit:
                logger.warning(f'WARNING: NMS time limit {time_limit}s exceeded')
                break

        return output

    # ------------------------------------------------------------------
    # Segmentation / visualisation

    @staticmethod
    def driving_area_mask(seg=None):
        """Extract driving-area segmentation mask from model output."""
        da_predict = seg[:, :, 12:372, :]
        da_seg_mask = torch.nn.functional.interpolate(
            da_predict, scale_factor=2, mode='bilinear'
        )
        _, da_seg_mask = torch.max(da_seg_mask, 1)
        return da_seg_mask.int().squeeze().cpu().numpy()

    @staticmethod
    def lane_line_mask(ll=None):
        """Extract lane-line segmentation mask from model output."""
        ll_predict = ll[:, :, 12:372, :]
        ll_seg_mask = torch.nn.functional.interpolate(
            ll_predict, scale_factor=2, mode='bilinear'
        )
        ll_seg_mask = torch.round(ll_seg_mask).squeeze(1)
        return ll_seg_mask.int().squeeze().cpu().numpy()

    @staticmethod
    def show_seg_result(img, result, palette=None, is_demo=False):
        """Overlay segmentation masks onto *img* in-place."""
        if palette is None:
            palette = np.random.randint(0, 255, size=(3, 3))
        palette[0] = [0, 0, 0]
        palette[1] = [0, 255, 0]
        palette[2] = [255, 0, 0]
        palette = np.array(palette)

        if not is_demo:
            color_seg = np.zeros((result.shape[0], result.shape[1], 3), dtype=np.uint8)
            for label, color in enumerate(palette):
                color_seg[result == label, :] = color
        else:
            color_area = np.zeros(
                (result[0].shape[0], result[0].shape[1], 3), dtype=np.uint8
            )
            color_area[result[0] == 1] = [0, 255, 0]
            color_area[result[1] == 1] = [255, 0, 0]
            color_seg = color_area

        color_seg = color_seg[..., ::-1]
        color_mask = np.mean(color_seg, 2)
        img[color_mask != 0] = img[color_mask != 0] * 0.5 + color_seg[color_mask != 0] * 0.5

    @staticmethod
    def plot_one_box(x, img, color=None, label=None, line_thickness=3):
        """Draw a single bounding box on *img* in-place."""
        tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, [0, 255, 255], thickness=2, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)
            cv2.putText(
                img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255],
                thickness=tf, lineType=cv2.LINE_AA,
            )
